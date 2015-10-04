classdef trajectoryFollower
    properties
        controller;
    end
    methods(Static = true)
        function obj = trajectoryFollower(controller)
            obj.controller = controller;
        end
        function feedForward()
            %time thing********************** FIX THIS
            feedback = true; %boolean deciding to add feedback factor

            %need this for data logs later?
            timeArray = zeros(1,1);
            distArray = zeros(1,1); % create 1x1 matrix with value 0
       
            leftStart = double(robot.encoders.LatestMessage.Left);
            leftEncoder = double(leftStart); % in "mm"

            signedDistance =0;
            arrayIndex = 1; %index starts with 1

            time=0;
         
            fig_ref = figure8ReferenceControl(0.5,0.4,0.5); 
            ti = 0;
            tf = fig_ref.getTrajectoryDuration(fig_ref);
            dt = .01;
            robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, fig_ref);
            
            while time < (tf)
                elapsedTic = tic;
                pause(0.001);
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                time = time + elapsedTime;

                v_t = robotTraj.getVelocityAtTime(robotTraj,time);
                w_t = robotTraj.getOmegaAtTime(robotTraj,time);
               % [x;y;th] = robotTraj.getPoseAtTime(robotTraj,time);
                
                leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                signedDistance = double(double(leftEncoder - leftStart)/1000); %in m

                v_control = 0;
                w_control = 0;

                if(feedback)
                    pose_ref = robotTraj.getPoseAtTime(robotTraj,time);
                    %TODO: calculate pose_robot and robot_yaw from
                    %feedback? maybe make an array of feedback vl,vr and caculate
                    %from that
                    [v_control, w_control] = obj.controller.velFeedback(pose_robot, pose_ref, robot_yaw);
                end
                
                v_real = v_t + v_control;
                w_real = w_t + w_control;

                rM = robotModel();
                [vl, vr] = rM.VwTovlvr(rM,v_real,w_real);

                robot.sendVelocity(vl, vr);

                timeArray(arrayIndex) = time; %total elapsed time so far
                distArray(arrayIndex) = signedDistance; %total travelled distance so far
             
                arrayIndex = arrayIndex + 1;
            end

            robot.sendVelocity(0.0, 0.0);
            pause(1);

          %  figure(1);
          %  plot(timeArray, distArray, timeArray, sDelayArray);
          %  figure(2);
          %  plot(timeArray, distArray - sDelayArray);

            
        end
    end
end