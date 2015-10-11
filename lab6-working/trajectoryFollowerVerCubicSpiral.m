classdef trajectoryFollowerVerCubicSpiral
    properties
        controller;
        cubicSpiral;
    end
    methods(Static = true)
        function obj = trajectoryFollowerVerCubicSpiral(controller, cubicSpiral)
            obj.controller = controller;
            obj.cubicSpiral = cubicSpiral;
        end
        function feedForward(robot, obj, feedback, delay,tryNum)
            % follwer function get a traj from robot.traj and make the
            % robot follow thw path by gettting the correct vl and vr at
            % timn t and send it to the robot. if feedback is true, the
            % follwer will adjust its position according to the error
            % vector(defined as refer_pose - current_pose) using PID
            % control on x y and th. 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %init the data Logger(simply just arrays that records data at
            % a time t)
            timeArray = zeros(1,1); % moment t
            distArray = zeros(1,1); % dist travel at moment t
            vRealArray = zeros(1,1); % real linear speed at t
            wRealArray = zeros(1,1); % real angular  speed at t
            vlRealArray = zeros(1,1); % real vl speed at t
            vrRealArray = zeros(1,1); % real vr speed at t
            xRealArray = zeros(1,1); % x position at t
            yRealArray = zeros(1,1); % y position at t
            
            thRealArray = zeros(1,1); % angle at t
            xErrorArray = zeros(1,1);%x error from reference at time t
            yErrorArray = zeros(1,1);%y error from reference at time t
            thErrorArray = zeros(1,1);%th error from reference at time t
            errorArray = zeros(1,1);%legnth of error vector at time t
            
            %time to set up the things for robot to move
            
            % read the value from the encoder
            %encoder reads distance in mm, to get in m, divide by 1000
            leftLast = double(double(robot.encoders.LatestMessage.Left)/1000);
            leftNow = double(leftLast); 
            rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
            rightNow = double(rightLast);
          
            % distance the left and right wheels traveled in dt
            lds = 0;
            rds = 0;
            
            i = 2; %index starts with 2(because we know the init state
                   %of the robot)

            time=0;% our clock
         
            %get our reference trajertory 
   %         ti = 0;
            tf = getTrajectoryDuration(obj.cubicSpiral);
   %         dt = 1e-3;
   %         s_o = 0;
   %         p_o = [0; 0; 0];
   %         robotTraj = robotTrajectory(ti, tf, dt, s_o, p_o, obj.referenceControl);
            % get the model of robot(so that we get functions to compute vl
            % and vr from linear and angular velocity( V and w) or reverse)
            %rM = robotModel();
            %start the clock
            tic;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %myPlot = plot(xRealArray,yRealArray); %make real time plot of real 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  % robot traj
            while time < (tf)
                pause(0.001);
                time = toc; 
                %dt of this iteration(damn, dt is taken)
                dtime = time-timeArray(end);
                
                %get the V and w the robot should have at the time
                v_t = getVAtTime(obj.cubicSpiral,time+delay);
                w_t = getwAtTime(obj.cubicSpiral,time+delay);              
                %control parameters we need for PID control
                %(we just add them to V and w brutally and hope)
                v_control = 0;
                w_control = 0;
                
                % read encoder and update old reading
                %(of course, still in mm, divide by 1000)
                leftNow = double(double(robot.encoders.LatestMessage.Left)/1000); 
                lds = double(leftNow - leftLast);
                leftLast = double(leftNow);
                
                rightNow = double(double(robot.encoders.LatestMessage.Right)/1000);
                rds = double(rightNow - rightLast); 
                rightLast = double(rightNow);
                
                %get the real vl and vr
                real_vl = double(double(lds)/dtime); 
                real_vr = double(double(rds)/dtime);
                [V,w] = robotModel.vlvrToVw(real_vl,real_vr);
                %disp(V);
                %disp(w);
                dth = w*dtime;
                %last angle the robot is pointing
                th = thRealArray(i-1);
                %the pose[x; y; th] robot should be at the time
                ref_pose = getPoseAtTime(obj.cubicSpiral,time);
                
                %update the data logger(just arrays)
                vlRealArray(i) = real_vl;
                vrRealArray(i) =  real_vr;
                thRealArray(i) = thRealArray(i-1)+dth;
                xRealArray(i) = xRealArray(i-1) + double(V*cos(th)*dtime);
                yRealArray(i) = yRealArray(i-1) + double(V*sin(th)*dtime);
                timeArray(i) = time;
                vRealArray(i) = V;
                wRealArray(i) = w;
                %angle robot is now pointing
                robot_th = thRealArray(i);
                robot_x = xRealArray(i);
                robot_y = yRealArray(i);
                robot_pose = [robot_x ;robot_y ;robot_th];
                %the matrix that transform vector from robot frame to world
                %frame
                RtoWMatrix = [cos(robot_th) -sin(robot_th) robot_x;
                              sin(robot_th)  cos(robot_th) robot_y;
                              0              0             1];

                WtoRMatrix = RtoWMatrix^-1; %inverse it we have world to robot
                % ref_x - real_x and ref_y - real_y we have our error
                % vector
                errorInWorldFrame = [ref_pose(1)-robot_pose(1);
                                     ref_pose(2)-robot_pose(2);
                                     1]; 
                %we need the error vector in robot frame           
                errorInRobotFrame = WtoRMatrix * errorInWorldFrame;
                xErrorArray(i) = errorInRobotFrame(1);
                yErrorArray(i) = errorInRobotFrame(2);
                errorArray(i) = sqrt(xErrorArray(i)^2+yErrorArray(i)^2);
                thErrorArray(i) =ref_pose(3)-robot_pose(3);

                if(feedback)
                    %get two control parameter from the controller
                   [v_control, w_control] =...
                   obj.controller.velFeedback(obj.controller,...
                   [errorInRobotFrame(1);errorInRobotFrame(2)]);
                end
                
                % we get the V and w robot should have, if feedback  is
                % true then PID is controling
                %(adding brutally)
                V_command = v_t + v_control;
                w_command = w_t + w_control;
                
                %from linear and angular speed, we have vl and vr
                [vl, vr] = robotModel.VwTovlvr(V_command,w_command);
                %avoid excedding robot max speed
                if(abs(vl)>0.3)
                    vl = 0.3;
                end
                if(abs(vr)>0.3)
                    vr = 0.3;
                end
                %finally, we can make the robot move at vr and vl
                robot.sendVelocity(vl, vr);
               
                %we move on to next iteration
                i = i + 1;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %set(myPlot, 'Xdata', xRealArray, 'Ydata', yRealArray);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %end
            end          
            % stop robot after completing the traj
            robot.sendVelocity(0.0, 0.0);
            pause(1);
            xErrorArray = xErrorArray/100;
            yErrorArray = yErrorArray/100;
            errorArray = errorArray/100;
            
            %here we start make the plots
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % real-data arrays are all one elemment more than theory
            %xArray = xArray(1:end-1);
            %yArray = yArray(1:end-1);
            %thArray = thArray(1:end-1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %plot(timeArray, poseXArray, timeArray, yRealArray, timeArray, thRealArray, timeArray, xArray, timeArray, yArray, timeArray, thArray);
            tryNum
            figure (tryNum);
            title('ref');
            hold on;
          %  plot(obj.cubicSpiral.poseArray(1,:),obj.cubicSpiral.poseArray(2,:),'-r',xRealArray,yRealArray,'-b');
            plot(obj.cubicSpiral.poseArray(1,:),obj.cubicSpiral.poseArray(2,:),'-b');
             plot(xRealArray,yRealArray,'-r');
            title('reference traj and real traj');
            hold off
            
      %      figure;
            %thErrorArray(thErrorArray > 10) = 0;
      %      plot(timeArray, xErrorArray);
      %      title('xError vs time');
      %      figure;
      %      plot(timeArray, yErrorArray);
      %      title('yError vs time');
      %      figure;
      %      plot(timeArray, errorArray);
      %      title('error vs time');
            %%*************
           % figure(5)
           % plot(timeArray,vlArray/10, timeArray,cvlArray);
           % a=1;
        end
    end
end