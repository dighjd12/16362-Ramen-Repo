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
        SE;
        
        robotStart;
    end
    methods(Static = true)
        function obj = trajectoryFollowerVerCubicSpiral(controller, SE)
            obj.controller = controller;
            obj.SE = SE;
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
            obj.robotStart = [0;0;0];
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
            
            leftLast = double(double(robot.encoders.LatestMessage.Left)/1000);
            leftNow = double(leftLast);
            rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
            rightNow = double(rightLast);
          
            % distance the left and right wheels traveled in dt
            lds = 0;
            rds = 0;
            i = 2; 
            
            obj.SE.i = 2;
            obj.SE.plidarArray = zeros(3,1);
                                           
            time= 0;% our clock
            
            obj.timeArray = zeros(1,1);
            obj.xRealArray = zeros(1,1);
            obj.yRealArray = zeros(1,1);
            obj.thRealArray = zeros(1,1);
            obj.xRefArray = zeros(1,1);
            obj.yRefArray = zeros(1,1);
            obj.thRefArray = zeros(1,1);
            obj.xErrorArray = zeros(1,1);
            obj.yErrorArray = zeros(1,1);
            obj.thErrorArray = zeros(1,1);
            obj.errorArray = zeros(1,1);
            obj.xpRealArray = zeros(1,1);
            obj.ypRealArray = zeros(1,1);
            obj.xpRefArray = zeros(1,1);
            obj.ypRefArray = zeros(1,1);
            obj.xpRealArray(1) = obj.robotStart(1);
            obj.ypRealArray(1) = obj.robotStart(2);
            obj.xpRefArray(1) = obj.robotStart(1);
            obj.ypRefArray(1) = obj.robotStart(2);
            
            odometryArray = zeros(3,1);
            fusedArray = zeros(3,1);
            odometryInRSArray = zeros(3,1);
            fusedInRSArray = zeros(3,1);
            
            obj.SE.setInitPose(obj.SE,[0;0;0]);
            
            fprintf('starting trajectory.. lastposer: %d %d %d\n' ...
                ,obj.lastPoser(1),obj.lastPoser(2),obj.lastPoser(3));
            fprintf('starting trajectory.. pf at the beginning: %d %d %d\n' ...
                ,obj.SE.poseFused(1),obj.SE.poseFused(2),obj.SE.poseFused(3));
           
              fprintf('starting trajectory.. starting pose in world frame: %d %d %d\n' ...
                ,obj.robotStart(1),obj.robotStart(2),obj.robotStart(3));
            
            rsTowMatrix = pose(obj.robotStart).bToA();
            fsTowMatrix = pose(obj.robotStart).bToA();
            rcTowMatrix = rsTowMatrix;
            rcTowMatrix2 = rsTowMatrix;
            tf = getTrajectoryDuration(obj.trajectory);

            tic;
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
                th = obj.thRealArray(i-1) + dth;
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
                
                dx = double(V*cos(th)*dtime);
                dy = double(V*sin(th)*dtime);
                dxc = double(V*cos(dth)*dtime);
                dyc = double(V*sin(dth)*dtime);
                
                
%                 obj.thRealArray(i) = obj.thRealArray(i-1) + dth;
%                 obj.xRealArray(i) = obj.xRealArray(i-1) + dx;
%                 obj.yRealArray(i) = obj.yRealArray(i-1) + dy;
                
                rcTowMatrix = rcTowMatrix * pose(dxc,dyc,dth).bToA(); %odometry
                obj.thRealArray(i) = obj.thRealArray(i-1) + dth;
                obj.xRealArray(i) = obj.xRealArray(i-1) + dx;
                obj.yRealArray(i) = obj.yRealArray(i-1) + dy;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                result1 = pose.matToPoseVec(rcTowMatrix); %lastposer?
                result2 = pose.matToPoseVec(fsTowMatrix * pose([obj.xRefArray(i);obj.yRefArray(i);obj.thRefArray(i)]).bToA());
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
            
                odometryInRSArray(:,i) = [robot_x ;robot_y ;robot_th];
                odometryArray(:,i) = pose.matToPoseVec(rsTowMatrix * pose([robot_x ;robot_y ;robot_th]).bToA());
                % in world frame
                
                robot_pose = obj.SE.fusePose(obj.SE, robot, dx, dy, dth, rsTowMatrix);
                robot_th = robot_pose(3);
                robot_x = robot_pose(1);
                robot_y = robot_pose(2);
            
                fusedInRSArray(:,i) = robot_pose;
                fusedArray(:,i) = pose.matToPoseVec(rsTowMatrix * pose(robot_pose).bToA());
                rcTowMatrix = rcTowMatrix * pose(dxc,dyc,dth).bToA();
                % in world frame
                
                %the matrix that transform vector from robot frame to world
                %frame
%                 RtoWMatrix = [cos(robot_th) -sin(robot_th) robot_x;
%                               sin(robot_th)  cos(robot_th) robot_y;
%                               0              0             1];
% 
%                 WtoRMatrix = RtoWMatrix^-1; %inverse it we have world to robot
%                 % ref_x - real_x and ref_y - real_y we have our error
%                 % vector
%                 errorInWorldFrame = [ref_pose(1)-robot_pose(1);
%                                      ref_pose(2)-robot_pose(2);
%                                      1]; 
%                 %we need the error vector in robot frame           
%                 errorInRobotFrame = WtoRMatrix * errorInWorldFrame;
%                 obj.xErrorArray(i) = errorInRobotFrame(1);
%                 obj.yErrorArray(i) = errorInRobotFrame(2);
%                 obj.errorArray(i) = sqrt(obj.xErrorArray(i)^2+obj.yErrorArray(i)^2);
%                 obj.thErrorArray(i) =ref_pose(3)-robot_pose(3);

                th2 = ref_pose(3);
                th1 = robot_pose(3);
                tdiff = atan2(sin(th2-th1),cos(th2-th1));
                errorVecInRSFrame = [ref_pose(1)-robot_pose(1);
                                  ref_pose(2)-robot_pose(2);
                                  tdiff];
                %we need the error vector in robot frame
                errorInRobotFrame = errorVecInRSFrame;
                obj.xErrorArray(i) = errorInRobotFrame(1);
                obj.yErrorArray(i) = errorInRobotFrame(2);
                obj.thErrorArray(i) = errorInRobotFrame(3);
                obj.errorArray(i) = sqrt(obj.xErrorArray(i)^2+obj.yErrorArray(i)^2);

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
                robot.sendVelocity(vl, vr);
               
                i = i + 1;
            end          
            % stop robot after completing the traj
            robot.sendVelocity(0.0, 0.0);
            pause(1);
            obj.xErrorArray = obj.xErrorArray/100;
            obj.yErrorArray = obj.yErrorArray/100;
            obj.errorArray = obj.errorArray/100;
            
            figure (2);
            hold on;
            plot(obj.xpRefArray,obj.ypRefArray,'-b');
            plot(obj.xpRealArray,obj.ypRealArray,'-r');
            title('reference traj, b, and real traj, r');
            hold off
            
            figure (3);
            title('x: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryArray(1,:),'-g');
            plot(obj.timeArray(1,:),obj.SE.plidarArray(1,:),'-b');
            plot(obj.timeArray(1,:),fusedArray(1,:),'-r');
            hold off
            figure (4);
            title('y: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryArray(2,:),'-g');
            plot(obj.timeArray(1,:),obj.SE.plidarArray(2,:),'-b');
            plot(obj.timeArray(1,:),fusedArray(2,:),'-r');
            hold off
            figure (5);
            title('th: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryArray(3,:),'-g');
            plot(obj.timeArray(1,:),obj.SE.plidarArray(3,:),'-b');
            plot(obj.timeArray(1,:),fusedArray(3,:),'-r');
            hold off
            
            figure (6);
            title('errorsInRobotStartFrame x: r, y: b, th: g');
            hold on;
            plot(obj.timeArray(1,:),obj.xErrorArray(1,:),'-r');
            plot(obj.timeArray(1,:),obj.yErrorArray(1,:),'-b');
            plot(obj.timeArray(1,:),obj.thErrorArray(1,:),'-g');
            hold off
            
            figure (7);
            title('x: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryInRSArray(1,:),'-g');
            plot(obj.timeArray(1,:),fusedInRSArray(1,:),'-r');
            hold off
            figure (8);
            title('y: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryInRSArray(2,:),'-g');
            plot(obj.timeArray(1,:),fusedInRSArray(2,:),'-r');
            hold off
            figure (9);
            title('th: pf-red, pl-blue, po-green');
            hold on;
            plot(obj.timeArray(1,:),odometryInRSArray(3,:),'-g');
            plot(obj.timeArray(1,:),fusedInRSArray(3,:),'-r');
            hold off
            disp('is this right??');
            disp(pose.matToPoseVec(rcTowMatrix));
            disp('xyth real array');
            realArrs = [obj.xRealArray(end);
                             obj.yRealArray(end);
                             obj.thRealArray(end)]
                         
                         
            obj.lastPoser = [obj.xpRealArray(end);
                             obj.ypRealArray(end);
                             obj.thRealArray(end)];
                         
            obj.lastPosef = [obj.xpRefArray(end);
                             obj.ypRefArray(end);
                             obj.thRefArray(end)];
        end
    end
end