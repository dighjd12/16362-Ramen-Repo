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
            poseXArray = zeros(1,1);%%************
            poseYArray = zeros(1,1);%%************
            yawArray = zeros(1,1);%%************
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xErrorArray = zeros(1,1);
            yErrorArray = zeros(1,1);
            thErrorArray = zeros(1,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %leftStart = double(robot.encoders.LatestMessage.Left);
            %leftEncoder = double(leftStart); % in "mm"

            %signedDistance =0;

            %distArray = zeros(1,1); % create 1x1 matrix with value 0
            vlArray = zeros(1,1); %left wheel speed array
            vrArray = zeros(1,1); %right wheel speed array
            thArray = zeros(1,1); %angle array 
            xArray = zeros(1,1); %world frame x array
            yArray = zeros(1,1); %world frame y array
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            m_xArray = zeros(1,1); % x array from the model
            m_yArray = zeros(1,1); %y array from the model
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
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
            p_o = [0; 0; 0];%%************
            counter = 1;
            errorXArray = 0;
            errorYArray = 0;
            errorThArray = 0;
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, obj.referenceControl);
            rM = robotModel();
            
            elapsedTic = tic;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %myPlot = plot(xArray,yArray);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            while time < (tf)
                pause(0.001);
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                elapsedTic = tic;
                time = time + elapsedTime;

                v_t = robotTraj.getVelocityAtTime(robotTraj,time);
                w_t = robotTraj.getOmegaAtTime(robotTraj,time);
                      
                v_control = 0;
                w_control = 0;
                
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
                dth = w*dtime;
                thArray(arrayIndex+1) = thArray(arrayIndex)+dth;
                th = thArray(arrayIndex);
                xArray(arrayIndex+1) = xArray(arrayIndex) + V*cos(th);
                yArray(arrayIndex+1) = yArray(arrayIndex) + V*sin(th);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                pose = robotTraj.getPoseAtTime(robotTraj,time);
                %POSE IS [ xref
                %          yref ]   of size (2,1) at time time. 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                yaw = robotTraj.getYawAtTime(robotTraj,time);%%************
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                robot_yaw = thArray(arrayIndex+1);
                rotMatrix = [cos(robot_yaw) -sin(robot_yaw); sin(robot_yaw) cos(robot_yaw)];
                pose_robot = [xArray(arrayIndex+1); yArray(arrayIndex+1)];
                errorInRobotFrame = ((rotMatrix^-1)*(pose - pose_robot))';
                xErrorArray(arrayIndex) = errorInRobotFrame(1);
                yErrorArray(arrayIndex) = errorInRobotFrame(2);
                thErrorArray(arrayIndex) = (yaw - robot_yaw)*1000;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                if(feedback)
                    [v_control, w_control] = obj.controller.velFeedback(obj.controller, pose_robot, pose, yaw, robot_yaw);%%************
                end
                
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
                vrealArray(arrayIndex) = v_real;
                wrealArray(arrayIndex) = w_real;
                poseXArray(arrayIndex) = pose(1); %x points
                poseYArray(arrayIndex) = pose(2); %y points
                yawArray(arrayIndex) = yaw;%%************
                arrayIndex = arrayIndex + 1;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %set(myPlot, 'Xdata', xArray, 'Ydata', yArray);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %error = ((xArray(end)-xArray(1))^2+((yArray(end)-yArray(1))^2))^(0.5);
            %disp(xArray(end));
            %disp(yArray(end));
            %disp(error);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            robot.sendVelocity(0.0, 0.0);
            pause(1);
            
            %%************
            figure(2);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % real-data arrays are all one elemment more than theory
            xArray = xArray(1:end-1);
            yArray = yArray(1:end-1);
            thArray = thArray(1:end-1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            plot(timeArray, poseXArray, timeArray, poseYArray, timeArray, yawArray, timeArray, xArray, timeArray, yArray, timeArray, thArray);
            figure(3);
            hold on;
            plot(robotTraj.x,robotTraj.y);
            plot(xArray/200,yArray/200);
            title('reference plot');
            hold off;
            figure(4);
            thErrorArray(thErrorArray > 10) = 0;
            plot(timeArray, xErrorArray, timeArray, yErrorArray, timeArray, thErrorArray);
            %%*************
            a=1;
        end
    end
end