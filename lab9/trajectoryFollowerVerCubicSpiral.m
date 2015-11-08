classdef trajectoryFollowerVerCubicSpiral < handle
    properties   
        controller;
        SE;
        % below are data loggers(just arrays)
        timeArray;
        distArray;
        vRealArray;
        wRealArray;
        vlRealArray;
        vrRealArray;     % [vl, vr] real speeds    at time t
        realArray;      % x, y, th position/angle at time t
        errorArray;     % x, y, th error from ref at time t
        errorSizeArray; % length of error vector  at time t
        refArray;       % x, y, th reference      at time t
 
        xpRealArray;
        ypRealArray;
        xpRefArray;
        ypRefArray;
          
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lastIndex;
        trajectory;
        startPose;
        lastPoser;
        lastPosef;
    end
    methods(Static = true)
        function obj = trajectoryFollowerVerCubicSpiral(controller,SE)
            obj.controller = controller;
            obj.SE = SE;
            %obj.cubicSpiral = cubicSpiral;
            obj.timeArray = zeros(1,1); % moment t
            obj.distArray = zeros(1,1); % dist travel at moment t
            obj.vRealArray = zeros(1,1); % real linear speed at t
            obj.wRealArray = zeros(1,1); % real angular  speed at t
            obj.vlRealArray = zeros(1,1); % real vl speed at t
            obj.vrRealArray = zeros(1,1); % real vr speed at t
            
            obj.realArray = zeros(1,3); % x, y, th position/angle at t       
            obj.errorArray = zeros(1,3); % x, y, th error from ref at time t           
            obj.refArray = zeros(1,3);   % x, y, th reference at time t

            obj.errorSizeArray = zeros(1,1); %length of error vector at time t
            obj.xpRealArray = zeros(1,1);
            obj.ypRealArray = zeros(1,1);
            obj.xpRefArray = zeros(1,1);
            obj.xpRefArray = zeros(1,1);
            
            obj.lastPoser = [0;0;0];
            obj.lastPosef = [0;0;0];
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
                           0                    ,  0                    ,1];
            fsTowMatrix = [cos(obj.lastPosef(3)), -sin(obj.lastPosef(3)),obj.lastPosef(1);
                           sin(obj.lastPosef(3)),  cos(obj.lastPosef(3)),obj.lastPosef(2);
                           0,0,1];
                      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
            tf = getTrajectoryDuration(obj.trajectory);
            %start the clock
            tic;
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
                dth = w*dtime;
                %last angle the robot is pointing
                th = obj.realArray(i-1,3);
                %the pose[x; y; th] robot should be at the time
                ref_pose = getPoseAtTime(obj.trajectory,time);
                obj.refArray(i,:) = ref_pose;
                
                %update the data logger(just arrays)
                obj.vlRealArray(i) = real_vl;
                obj.vrRealArray(i) =  real_vr;
                obj.realArray(i,3) = obj.realArray(i-1,3)+dth;
                dx = double(V*cos(th)*dtime);
                dy = double(V*sin(th)*dtime);
                obj.realArray(i,1) = obj.realArray(i-1,1) + dx;
                obj.realArray(i,2) = obj.realArray(i-1,2) + dy;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                result1 = rsTowMatrix * [obj.realArray(i,1);obj.realArray(i,2);1];
                result2 = fsTowMatrix * [obj.refArray(i,1) ;obj.refArray(i,2) ;1];
                obj.xpRealArray(i) = result1(1);
                obj.ypRealArray(i) = result1(2);
                obj.xpRefArray(i) = result2(1);
                obj.ypRefArray(i) = result2(2);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.timeArray(i) = time;
                obj.vRealArray(i) = V;
                obj.wRealArray(i) = w;
                
                %angle robot is now pointing
                robot_th = obj.realArray(i,3);
                robot_x  = obj.realArray(i,1);
                robot_y  = obj.realArray(i,2);
                
             %   robot_x  = robot_pose(1);
             %   robot_y  = robot_pose(2);
             %   robot_th = robot_pose(3);
                
                %the matrix that transform vector from robot frame to world
                %frame
                RtoSMatrix = [cos(robot_th) -sin(robot_th) robot_x;
                              sin(robot_th)  cos(robot_th) robot_y;
                              0              0             1];
                          
                result = rsTowMatrix * RtoSMatrix * [dx;dy;1];
                
                % robot_pose : fused_pose
                robot_pose = obj.SE.fusePose(obj.SE, robot, result(1),result(2),dth);
                          
                          
                StoRMatrix = RtoSMatrix^-1; %inverse it we have world to robot
                % ref_x - real_x and ref_y - real_y we have our error
                % vector
                errorInWorldFrame = [ref_pose(1)-robot_pose(1);
                                     ref_pose(2)-robot_pose(2);
                                     1]; 
                %we need the error vector in robot frame           
                errorInRobotFrame     = StoRMatrix * errorInWorldFrame;
                obj.errorArray(i,1)   = errorInRobotFrame(1);
                obj.errorArray(i,2)   = errorInRobotFrame(2);
                obj.errorSizeArray(i) = sqrt(obj.errorArray(i,1)^2+obj.errorArray(i,1)^2);
                obj.errorArray(i,3)     = ref_pose(3) - robot_pose(3);

                if(feedback)
                    %get two control parameter from the controller
                   [v_control, w_control] =...
                   obj.controller.velFeedback(obj.controller...
                   ,obj.errorArray(:,1)...
                   ,obj.errorArray(:,2)...
                   ,obj.errorArray(:,3)...
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
                %end
            end          
            % stop robot after completing the traj
            
            robot.sendVelocity(0.0, 0.0);
            pause(1);
            obj.errorArray(:,1) = obj.errorArray(:,1)/100;
            obj.errorArray(:,2) = obj.errorArray(:,2)/100;
            obj.errorSizeArray = obj.errorSizeArray/100;
            %here we start make the plots
            figure (2);
            title('ref');
            hold on;
            plot(obj.xpRefArray,obj.ypRefArray,'-b');
            plot(obj.xpRealArray,obj.ypRealArray,'-r');
            title('reference traj and real traj');
            hold off
            disp(double(sum(obj.errorSizeArray))/double(length(obj.errorSizeArray)));
              
            obj.lastPoser = [obj.xpRealArray(end);
                             obj.ypRealArray(end);
                             obj.thRealArray(end)];
        
            obj.lastPosef = [obj.xpRefArray(end);
                             obj.ypRefArray(end);
                             obj.thRefArray(end)];
    
        end
    end
end