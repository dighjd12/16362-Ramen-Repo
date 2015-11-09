classdef trajectoryFollowerVerCubicSpiral < handle
    properties   
        controller;
        % below are data loggers(just arrays)
        timeArray;
        distArray;
        vRealArray;
        wRealArray;
        vlRealArray;
        vrRealArray;
        xRealArray;
        yRealArray;        
        thRealArray;
        xRefArray;
        yRefArray;
        thRefArray;
        xErrorArray;
        yErrorArray;
        thErrorArray;
        xpRealArray;
        ypRealArray;
        xpRefArray;
        ypRefArray;
        errorArray;   
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lastIndex;
        trajectory;
        startPose;
        lastPoser;
        lastPosef;
    end
    methods(Static = true)
        function obj = trajectoryFollowerVerCubicSpiral(controller)
            obj.controller = controller;
            %obj.cubicSpiral = cubicSpiral;
            obj.timeArray = zeros(1,1); % moment t
            obj.distArray = zeros(1,1); % dist travel at moment t
            obj.vRealArray = zeros(1,1); % real linear speed at t
            obj.wRealArray = zeros(1,1); % real angular  speed at t
            obj.vlRealArray = zeros(1,1); % real vl speed at t
            obj.vrRealArray = zeros(1,1); % real vr speed at t
            obj.xRealArray = zeros(1,1); % x position at t
            obj.yRealArray = zeros(1,1); % y position at t      
            obj.thRealArray = zeros(1,1); % angle at t
            obj.xErrorArray = zeros(1,1);%x error from reference at time t
            obj.yErrorArray = zeros(1,1);%y error from reference at time t
            obj.thErrorArray = zeros(1,1);%th error from reference at time t
            
            obj.xRefArray = zeros(1,1);
            obj.yRefArray = zeros(1,1);
            obj.thRefArray = zeros(1,1);
            obj.errorArray = zeros(1,1);%legnth of error vector at time t
            obj.xpRealArray = zeros(1,1);
            obj.ypRealArray = zeros(1,1);
            obj.xpRefArray = zeros(1,1);
            obj.xpRefArray = zeros(1,1);
            
            obj.lastPoser = [0;0;0];
            obj.lastPosef = [0;0;0];
            obj.lastIndex = 1;
        end
        
        function obj = loadTrajectory(obj,trajectory, startPose)
            
            obj.trajectory = trajectory;
            obj.startPose = startPose;
            obj.controller.initialize(obj.controller,startPose);
        end
        
        function feedForward(robot, obj, feedback, delay)
            % follwer function get a traj from robot.traj and make the
            % robot follow thw path by gettting the correct vl and vr at
            % timn t and send it to the robot. if feedback is true, the
            % follwer will adjust its position according to the error
            % vector(defined as refer_pose - current_pose) using PID
            % control on x y and th. 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
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
                
            i = 2; %index starts with 
                                           %(because we know the init state
                                           %of the robot

            time= 0;% our clock
            obj.xpRealArray(1) = obj.lastPoser(1);
            obj.ypRealArray(1) = obj.lastPoser(2);
            obj.xpRefArray(1) = obj.lastPosef(1);
            obj.ypRefArray(1) = obj.lastPosef(2);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            rsTowMatrix = [cos(obj.lastPoser(3)), -sin(obj.lastPoser(3)),obj.lastPoser(1);
                           sin(obj.lastPoser(3)),  cos(obj.lastPoser(3)),obj.lastPoser(2);
                           0                ,  0                ,1];
            fsTowMatrix = [cos(obj.lastPosef(3)), -sin(obj.lastPosef(3)),obj.lastPosef(1);
                           sin(obj.lastPosef(3)),  cos(obj.lastPosef(3)),obj.lastPosef(2);
                           0,0,1];
            %rsTowMatrix = rwTosMatrix^-1;
            %disp(rsTowMatrix);
                      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %get our reference trajertory 
   %         ti = 0;
             
            tf = getTrajectoryDuration(obj.trajectory);
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
                dtime = time-obj.timeArray(end);
                
                %get the V and w the robot should have at the time
                v_t = getVAtTime(obj.trajectory,time+delay);
                w_t = getwAtTime(obj.trajectory,time+delay);              
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
                th = obj.thRealArray(i-1);
                %the pose[x; y; th] robot should be at the time
                ref_pose = getPoseAtTime(obj.trajectory,time);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.xRefArray(i) = ref_pose(1);
                obj.yRefArray(i) = ref_pose(2);
                obj.thRefArray(i) = ref_pose(3);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                
                %update the data logger(just arrays)
                obj.vlRealArray(i) = real_vl;
                obj.vrRealArray(i) =  real_vr;
                obj.thRealArray(i) = obj.thRealArray(i-1)+dth;
                obj.xRealArray(i) = obj.xRealArray(i-1) + double(V*cos(th)*dtime);
                obj.yRealArray(i) = obj.yRealArray(i-1) + double(V*sin(th)*dtime);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                result1 = rsTowMatrix  * [obj.xRealArray(i);obj.yRealArray(i);1];
                result2 = fsTowMatrix * [obj.xRefArray(i);obj.yRefArray(i);1];
                obj.xpRealArray(i) = result1(1);
                obj.ypRealArray(i) = result1(2);
                obj.xpRefArray(i) = result2(1);
                obj.ypRefArray(i) = result2(2);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.timeArray(i) = time;
                obj.vRealArray(i) = V;
                obj.wRealArray(i) = w;
                %angle robot is now pointing
                robot_th = obj.thRealArray(i);
                robot_x = obj.xRealArray(i);
                robot_y = obj.yRealArray(i);
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
                obj.xErrorArray(i) = errorInRobotFrame(1);
                obj.yErrorArray(i) = errorInRobotFrame(2);
                obj.errorArray(i) = sqrt(obj.xErrorArray(i)^2+obj.yErrorArray(i)^2);
                obj.thErrorArray(i) =ref_pose(3)-robot_pose(3);

                if(feedback)
                    %get two control parameter from the controller
                   obj.controller.started = true;
                   [v_control, w_control] =...
                   obj.controller.velFeedback(obj.controller...
                   ,obj.xErrorArray...
                   ,obj.yErrorArray...
                   ,obj.thErrorArray...
                   ,dtime);
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
            %disp(obj.xRealArray(1));
            robot.sendVelocity(0.0, 0.0);
            pause(1);
            obj.xErrorArray = obj.xErrorArray/100;
            obj.yErrorArray = obj.yErrorArray/100;
            obj.errorArray = obj.errorArray/100;
            %here we start make the plots
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % real-data arrays are all one elemment more than theory
            %xArray = xArray(1:end-1);
            %yArray = yArray(1:end-1);
            %thArray = thArray(1:end-1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %plot(timeArray, poseXArray, timeArray, yRealArray, timeArray, thRealArray, timeArray, xArray, timeArray, yArray, timeArray, thArray);
            figure (2);
            title('ref');
            hold on;
          %  plot(obj.cubicSpiral.poseArray(1,:),obj.cubicSpiral.poseArray(2,:),'-r',xRealArray,yRealArray,'-b');
            %plot(obj.xpRefArray,obj.ypRefArray,'-b');
            plot(obj.xpRealArray,obj.ypRealArray,'-r');
            title('reference traj and real traj');
            hold off
            disp(double(sum(obj.errorArray))/double(length(obj.errorArray)));
              
            obj.lastPoser = [obj.xpRealArray(end);
                  obj.ypRealArray(end);
                  obj.thRealArray(end)];
            obj.lastPosef = [obj.xpRefArray(end);
                  obj.ypRefArray(end);
                  obj.thRefArray(end)];
            
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