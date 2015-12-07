% Class:        trajectoryFollowerVerCubicSpiral.m
% Description:  generates and executes trajectory of a cubic spiral
% Last Edit:    11/09/2015

classdef trajectoryFollowerVerCubicSpiral < handle
    properties   
        % other objects
        controller;
        SE;
        
        % below are data loggers(just arrays)
        timeArray;
        distArray;
        vRealArray;     % linear (v) and angular (w) velocity at time t
        wRealArray;
        vlRealArray;    % vl & vr real speeds    at time t
        vrRealArray;    
        realArray;      % x, y, th position/angle at time t
        errorArray;     % x, y, th error from ref at time t
        errorSizeArray; % length of error vector  at time t
        refArray;       % x, y, th reference      at time t
 
        xpRealArray;
        ypRealArray;
        xpRefArray;
        ypRefArray;
        thpRealArray;
        thpRefArray;
          
        robotPoseArray;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lastIndex;
        trajectory;
        startPose;
        lastPoser;
        lastPosef;
        goalPose;
    end
    methods(Static = true)
        % Function:     trajectoryFollowerVerCubicSpiral
        % Description:  initialize object.  Zeros data logging arrays
        % Takes:        
        %   controller    - controller object
        %   SE            - stateEstimator object
        % Returns:      
        %   obj           - updated trajectory follower object
        % Last Edit:    11/09/2015
        function obj = trajectoryFollowerVerCubicSpiral(controller,SE)
            obj.controller = controller;
            obj.SE = SE;
            
            obj.timeArray   = zeros(1,1); % moment t
            obj.distArray   = zeros(1,1); % dist travel at moment t
            obj.vRealArray  = zeros(1,1); % real linear speed at t
            obj.wRealArray  = zeros(1,1); % real angular  speed at t
            obj.vlRealArray = zeros(1,1); % real vl speed at t
            obj.vrRealArray = zeros(1,1); % real vr speed at t
            
            obj.realArray   = zeros(1,3); % x, y, th position/angle at t       
            obj.errorArray  = zeros(1,3); % x, y, th error from ref at time t           
            obj.refArray    = zeros(1,3); % x, y, th reference at time t

            obj.errorSizeArray = zeros(1,1); %length of error vector at time t
            
            %these six keeps track of where robot is in the world frame
            obj.xpRealArray    = zeros(1,1); 
            obj.ypRealArray    = zeros(1,1);
            obj.thpRealArray = zeros(1,1);
            obj.xpRefArray     = zeros(1,1); 
            obj.ypRefArray     = zeros(1,1);
            obj.thpRefArray = zeros(1,1);
            
            obj.robotPoseArray = zeros(1,3);
            obj.lastPoser = [0;0;0];
            obj.lastPosef = [0;0;0];
            obj.goalPose = [0;0;0];
        end
        
        % Function:     loadTrajectory
        % Description:  initializes trajectory
        % Takes:        
        %   trajectory    - desired trajectory (
        %   startPose     - starting position of the robot
        % Returns:      
        %   obj           - updated trajectory follower object
        % Last Edit:    11/09/2015
        function obj = loadTrajectory(obj,trajectory, startPose)    
            obj.trajectory = trajectory;
            obj.startPose  = startPose;
            obj.controller.initialize(obj.controller,startPose);
        end
        
        % Function:     feedForward
        % Description:
        %       follower function get a traj from robot.traj and make the
        %       robot follow thw path by gettting the correct vl and vr at
        %       timn t and send it to the robot. if feedback is true, the
        %       follwer will adjust its position according to the error
        %       vector(defined as refer_pose - current_pose) using PID
        %       control on x y and th. 
        % Takes:        
        %   robot         - robot object
        %   feedback      - boolean to use or not use feedback
        %   delay         - delay between movement and reference
        % Returns:      
        %   none
        % Last Edit:    11/09/2015       
        function feedForward(robot, obj, feedback, delay,use_se)%,origin)
            
            %time to set up the things for robot to move
            
            % read the value from the encoder
            %get distance travel from encoder in meter(encoder reads
            %in mm)
            leftLast  = double(double(robot.encoders.LatestMessage.Left)/1000);
            leftNow   = double(leftLast); 
            rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
            rightNow  = double(rightLast);
          
            % distance the left and right wheels traveled in dt
            lds = 0;
            rds = 0;
                
            i = 2; %index starts with 2 (because we know the init state of the robot)

            time= 0;% our clock
            obj.xpRealArray = zeros(1,1);
            obj.ypRealArray = zeros(1,1);
            obj.xpRefArray  = zeros(1,1);
            obj.ypRefArray  = zeros(1,1);
            obj.thpRealArray = zeros(1,1);
            obj.thpRefArray = zeros(1,1);
            
            obj.xpRealArray(1) = obj.lastPoser(1);
            obj.ypRealArray(1) = obj.lastPoser(2);
            obj.xpRefArray(1)  = obj.lastPosef(1);
            obj.ypRefArray(1)  = obj.lastPosef(2);
            obj.thpRealArray(1) = obj.lastPoser(3);
            obj.thpRefArray(1) = obj.lastPosef(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %matrix that convert from traj start frame to world frame
            rsInw = pose(obj.lastPoser);
            refsInw = pose(obj.lastPosef); 
            obj.lastPosef = obj.goalPose;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
            tf = getTrajectoryDuration(obj.trajectory);
            %start the clock
            tic;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  % robot traj
            while time < (tf+delay)
                pause(0.001);
                time  = toc; 
                %dt of this iteration(damn, dt is taken)
                dtime = time - obj.timeArray(end);
                
                %get the V and w the robot should have at the time
                v_t = getVAtTime(obj.trajectory,time+delay);
                w_t = getwAtTime(obj.trajectory,time+delay);   
                
                %control parameters we need for PID control
                %(we just add them to V and w brutally and hope)
                v_control = 0;
                w_control = 0;
                
                % read encoder and update old reading
                %(of course, still in mm, divide by 1000)
                leftNow  = double(double(robot.encoders.LatestMessage.Left)/1000); 
                lds      = double(leftNow - leftLast);
                leftLast = double(leftNow);
                
                rightNow  = double(double(robot.encoders.LatestMessage.Right)/1000);
                rds       = double(rightNow - rightLast); 
                if abs(lds) > 0.4 || abs(rds) > 0.4
                    continue;
                end
                rightLast = double(rightNow);
                leftLast = double(leftNow);
                
                
                %get the real vl and vr
                real_vl = double(double(lds)/dtime); 
                real_vr = double(double(rds)/dtime);
                [V,w]   = robotModel.vlvrToVw(real_vl,real_vr);
                dth     = w*dtime;
                %last angle the robot is pointing
                th = obj.realArray(i-1,3);
                %the pose[x; y; th] robot should be at the time
                ref_pose          = getPoseAtTime(obj.trajectory,time+delay); %in robot start frame
                obj.refArray(i,:) = ref_pose;
                
                %update the data logger(just arrays)
                obj.vlRealArray(i) = real_vl;
                obj.vrRealArray(i) =  real_vr;
                obj.realArray(i,3) = obj.realArray(i-1,3)+dth;
                dx = double(V*cos(th)*dtime);
                dy = double(V*sin(th)*dtime);
                obj.realArray(i,1) = obj.realArray(i-1,1) + dx;
                obj.realArray(i,2) = obj.realArray(i-1,2) + dy;
                robot_th = obj.realArray(i,3);
                robot_x  = obj.realArray(i,1);
                robot_y  = obj.realArray(i,2);
                robot_pose = [robot_x;robot_y;robot_th];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % we convert the ref pose from robot frame to world frame
                refInRefs = pose(ref_pose);
                refPoseInw = pose.matToPoseVec(refsInw.bToA()*refInRefs.bToA());
                obj.xpRefArray(i) = refPoseInw(1);
                obj.ypRefArray(i) = refPoseInw(2);
                obj.thpRefArray(i) = refPoseInw(3);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                rInrs = pose(robot_pose);
                rPoseInw = pose.matToPoseVec(rsInw.bToA()*rInrs.bToA() );
                obj.xpRealArray(i) = rPoseInw(1);
                obj.ypRealArray(i) = rPoseInw(2);
                obj.thpRealArray(i) = rPoseInw(3);
  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.timeArray(i)  = time;
                obj.vRealArray(i) = V;
                obj.wRealArray(i) = w;
                
                %fprintf('\npose from odo:%d %d %d',rPoseInw(1),rPoseInw(2),rPoseInw(3));
               if use_se == true
                   rPoseInw = obj.SE.fusePose(obj.SE,robot,rPoseInw(1),rPoseInw(2),...
                                                 rPoseInw(3));
                  % fprintf('pose from fuse:%d %d %d\n',rPoseInw(1),rPoseInw(2),rPoseInw(3));
                   rInw = pose(rPoseInw);
                                         
                    rPoseInrs = pose.matToPoseVec(rsInw.aToB()*rInw.bToA());
                    robot_pose = rPoseInrs;
               end
              % disp(robot_pose);
               errorInStartFrame = [ref_pose(1)-robot_pose(1);
                                    ref_pose(2)-robot_pose(2);
                                    1                       ]; 
                %we need the error vector in robot frame           
                errorInRobotFrame     = (rInrs.bToA())^-1 * errorInStartFrame;
                obj.errorArray(i,1)   = errorInRobotFrame(1);
                obj.errorArray(i,2)   = errorInRobotFrame(2);
                obj.errorSizeArray(i) = sqrt(obj.errorArray(i,1)^2+obj.errorArray(i,1)^2);
                obj.errorArray(i,3)   = atan2(cos(ref_pose(3) - robot_pose(3)),...
                                             cos(ref_pose(3) - robot_pose(3)));
                errorInRobotFrame(3) = atan2(cos(ref_pose(3) - robot_pose(3)),...
                                          cos(ref_pose(3) - robot_pose(3)));
                if(feedback)
                    %get two control parameter from the controller
                    obj.controller.started = true;
                   [v_control, w_control] =...
                        obj.controller.velFeedback(obj.controller,...
                        errorInRobotFrame);
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
            obj.errorSizeArray  = obj.errorSizeArray/100;
            
            %here we start make the plots
            figure (2);
            title('ref');
            hold on;
            plot(obj.xpRefArray,obj.ypRefArray,'-b');
            plot(obj.xpRealArray,obj.ypRealArray,'-r');
            title('reference traj and real traj');
            %legend('Reference', 'Real');
            hold off
            fprintf('last pose in real traj is (%d %d %d)\n', obj.xpRealArray(end),...
            obj.ypRealArray(end),obj.thpRealArray(end));
        
            fprintf('last pose in ref traj is (%d %d %d)\n', obj.xpRefArray(end),...
            obj.ypRefArray(end),obj.thpRefArray(end));
            %fprintf('last pose in ref traj in robot frame is (%d %d %d)\n', ref_pose(1),...
           % ref_pose(2),ref_pose(3));
        
            obj.lastPoser = [obj.xpRealArray(end);
                             obj.ypRealArray(end);
                             obj.thpRealArray(end)];
            obj.lastPosef = [obj.xpRealArray(end);
                             obj.ypRealArray(end);
                             obj.thpRealArray(end)];
            
%              obj.lastPosef = [obj.xpRefArray(end);
%                               obj.ypRefArray(end);
%                               obj.thpRefArray(end)];
    
            disp('end of trajectory');
        end
    end
end