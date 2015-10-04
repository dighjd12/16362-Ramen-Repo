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
        function feedForward()
            
            feedback = true; %boolean deciding to add feedback factor

            %need this for data logs later?
            timeArray = zeros(1,1);
<<<<<<< Updated upstream
            %distArray = zeros(1,1); % create 1x1 matrix with value 0
            vrealArray = zeros(1,1);
            wrealArray = zeros(1,1);
            poseArray = zeros(1,1);
            pose2Array = zeros(1,1);
            
            %leftStart = double(robot.encoders.LatestMessage.Left);
            %leftEncoder = double(leftStart); % in "mm"

            %signedDistance =0;
=======
            distArray = zeros(1,1); % create 1x1 matrix with value 0
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
>>>>>>> Stashed changes
            arrayIndex = 1; %index starts with 1

            time=0;
         
<<<<<<< Updated upstream
=======
            fig_ref = figure8ReferenceControl(0.5,0.4,0.5); 
            %fig_ref = trapezoidalStepReferenceControl(.5,.75,.25,1, 1);
>>>>>>> Stashed changes
            ti = 0;
            tf = obj.referenceControl.getTrajectoryDuration(obj.referenceControl);
            dt = .01;
<<<<<<< Updated upstream
            s_o = 0;
            p_o = [0; 0];
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, obj.referenceControl);
=======
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, fig_ref);
>>>>>>> Stashed changes
            rM = robotModel();
            
            elapsedTic = tic;
            while time < (tf)
                pause(0.001);
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                elapsedTic = tic;
                time = time + elapsedTime;

                v_t = robotTraj.getVelocityAtTime(robotTraj,time);
                w_t = robotTraj.getOmegaAtTime(robotTraj,time);
<<<<<<< Updated upstream
               
                %leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                %signedDistance = double(double(leftEncoder - leftStart)/1000); %in m
=======
               % [x;y;th] = robotTraj.getPoseAtTime(robotTraj,time);
                
               % read encoder and upodate the start
                leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                leftDistance = double(double(leftEncoder - leftStart)/1000); %in meter
                leftStart = leftEncoder;
                
                rightEncoder = robot.encoders.LatestMessage.Right;
                rightDistance = double(double(rightEncoder - rightStart)/1000); %in meter
                rightStart = rightEncoder;
                
                
                %get the real pose of robot [x,y], th
                dt = elapsedTime; %get dt
                real_vl = double(leftDistance/dt); 
                real_vr = double(rightDistance/dt);
                vlArray(arrayIndex) = real_vl;
                vrArray(arrayIndex) =  real_vr;
                [V,w] = robotModel.vlvrToVw(robotModel,vl,vr);
                dth =  w*dt;
                thArray(arrayIndex+1) = thArray(arrayIndex)+dth;
                th = thArray(arrayIndex);
                xArray(arrayIndex+1) = xArray(arrayIndex) + V*cos(th);
                yArray(arrayIndex+1) = yArray(arrayIndex) + V*sin(th);
                robot_yaw = thArray(arrayIndex+1);
                pose_robot = [xArray(arrayIndex+1),yArray(arrayIndex+1)];               
>>>>>>> Stashed changes

                v_control = 0;
                w_control = 0;

                if(feedback)
   
                    pose_ref = robotTraj.getPoseAtTime(robotTraj,time);
                    %TODO: calculate pose_robot and robot_yaw from
                    %feedback? maybe make an array of feedback vl,vr and caculate
                    %from that
                    [v_control, w_control] = obj.controller.velFeedback(pose_robot, pose_ref, robot_yaw);
                end
                
                pose = robotTraj.getPoseAtTime(robotTraj,time);
                pose = pose';
                
                v_real = v_t + v_control;
                w_real = w_t + w_control;

                
                [vl, vr] = rM.VwTovlvr(rM,v_real,w_real);

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

            %figure(1);
            %plot(timeArray, vrealArray);
            %figure(2);
            %plot(timeArray, wrealArray);
            
        end
    end
end