classdef trajectoryFollower
    properties
        controller;
        referenceControl;
    end
    methods(Static = true)
        function obj = trajectoryFollower(controller, referenceControl)
            obj.controller = controller;
            obj.referenceControl = referenceControl;
        end
        function a = feedForward(robot, obj, feedback)
            
            %need this for data logs later?
            timeArray = zeros(1,1);
            %distArray = zeros(1,1); % create 1x1 matrix with value 0
            vrealArray = zeros(1,1);
            wrealArray = zeros(1,1);
            poseArray = zeros(1,1);
            pose2Array = zeros(1,1);
            
            %leftStart = double(robot.encoders.LatestMessage.Left);
            %leftEncoder = double(leftStart); % in "mm"

            %signedDistance =0;

            %distArray = zeros(1,1); % create 1x1 matrix with value 0
            vlArray = zeros(1,1); %left wheel speed array
            vrArray = zeros(1,1); %right wheel speed array
            thArray = zeros(1,1); %angle array 
            xArray = zeros(1,1); %world frame x array
            yArray = zeros(1,1); %world frame y array
            
            
            % read the value from the encoder
            leftStart = double(robot.encoders.LatestMessage.Left);
            leftEncoder = double(leftStart); % in "mm"
            rightStart = double(robot.encoders.LatestMessage.Right);
            rightEncoder = double(rightStart);
            
            % distance the left and right wheels traveled in dt
            leftDistance =0; 
            rightDistance = 0;
            arrayIndex = 1; %index starts with 1

            time=0;
         
            ti = 0;
            tf = obj.referenceControl.getTrajectoryDuration(obj.referenceControl);
            dt = .01;
            s_o = 0;
            p_o = [0; 0];
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, obj.referenceControl);
            rM = robotModel();
            
            elapsedTic = tic;
            while time < (tf)
                pause(0.001);
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                elapsedTic = tic;
                time = time + elapsedTime;

                v_t = robotTraj.getVelocityAtTime(robotTraj,time);
                w_t = robotTraj.getOmegaAtTime(robotTraj,time);
                      
                v_control = 0;
                w_control = 0;

                if(feedback)
   
                    pose_ref = robotTraj.getPoseAtTime(robotTraj,time);
                    %TODO: calculate pose_robot and robot_yaw from
                    %feedback? maybe make an array of feedback vl,vr and caculate
                    %from that
                    
                    % read encoder and upodate the start
                leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                leftDistance = double(double(leftEncoder - leftStart)/1000); %in meter
                leftStart = leftEncoder;
                
                rightEncoder = robot.encoders.LatestMessage.Right;
                rightDistance = double(double(rightEncoder - rightStart)/1000); %in meter
                rightStart = rightEncoder;
                                
                %get the real pose of robot [x,y], th
                dtime = elapsedTime; %get dt
                real_vl = double(leftDistance/dtime); 
                real_vr = double(rightDistance/dtime);
                vlArray(arrayIndex) = real_vl;
                vrArray(arrayIndex) =  real_vr;
                [V,w] = robotModel.vlvrToVw(robotModel,real_vl,real_vr);
                dth = w*dt;
                thArray(arrayIndex+1) = thArray(arrayIndex)+dth;
                th = thArray(arrayIndex);
                xArray(arrayIndex+1) = xArray(arrayIndex) + V*cos(th);
                yArray(arrayIndex+1) = yArray(arrayIndex) + V*sin(th);
                robot_yaw = thArray(arrayIndex+1);
                pose_robot = [xArray(arrayIndex+1); yArray(arrayIndex+1)];
                [v_control, w_control] = obj.controller.velFeedback(obj.controller, pose_robot, pose_ref, robot_yaw);
                end
                
                pose = robotTraj.getPoseAtTime(robotTraj,time);
                pose = pose';
                
                v_real = v_t + v_control;
                w_real = w_t + w_control;

                [vl, vr] = rM.VwTovlvr(rM,v_real,w_real);

                
                
                if(abs(vl)>0.3)
                    vl = 0.3;
                end
                if(abs(vr)>0.3)
                    vr = 0.3;
                end
                
                robot.sendVelocity(vl, vr);

                timeArray(arrayIndex) = time; %total elapsed time so far
               % distArray(arrayIndex) = signedDistance; %total travelled distance so far
                vrealArray(arrayIndex) = v_real;
                wrealArray(arrayIndex) = w_real;
                poseArray(arrayIndex) = pose(1); %x points
                pose2Array(arrayIndex) = pose(2); %y points
                arrayIndex = arrayIndex + 1;
            end

            robot.sendVelocity(0.0, 0.0);
            pause(1);

            a=1;
            %figure(1);
            %plot(timeArray, vrealArray);
            %figure(2);
            %plot(timeArray, wrealArray);
            
        end
    end
end