classdef mrplSystem < handle
    %HI this is main method 
    
    properties
        ctrl;
        follower;
        SE;
        lastStartPose;
    end
    
    methods(Static=true)
        function obj = mrplSystem()
            %obj.ctrl = controller(0.0001,0.0001,0.06);
            %obj.ctrl = controller(0.0005,0.00001,0.3);
            %obj.ctrl = controller(0.0001,0.000001,0.08);
            %obj.ctrl = controller(8e-3,1e-2,-3e-3);
            obj.ctrl = controller(8e-3,1e-2,3.8e-3);
            lines_p1 = [[0;0], [0;4]];
            lines_p2 = [[4;0], [0;0]];
            walls = [[4.0; 0.0], [0.0; 0.0], [0.0; 4.0]];
            mrplSystem.addStateEstimator(obj, 10, lines_p1, lines_p2, walls);
            obj.follower = trajectoryFollowerVerCubicSpiral(obj.ctrl, obj.SE);
            obj.lastStartPose = [0;0;0];
            
            
            
        end
        function addStateEstimator(obj, lidarSkip, lines_p1, lines_p2, worldLineArray)
            gain = 0.01;
            errThresh = 0.001;
            gradThresh = 0.0005;
            lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
            obj.SE = stateEstimator(lmLocalizer, worldLineArray, lidarSkip);
        end
        function setInitialPose(obj, startPose)
            obj.follower.lastPoser = startPose;
            obj.follower.lastPosef = startPose;
            obj.SE.setInitPose(obj.SE, obj.follower.lastPoser);
        end
        
        function executeTrajectorySE(obj,robot,xfa,yfa,thfa,sign)
            %make sure table is there.
            
            lastPose = obj.follower.lastPoser;
            obj.SE.setInitPose(obj.SE, obj.follower.lastPoser);
            rsInW = pose(lastPose);
            rfInW = pose(xfa,yfa,thfa);
            
            fprintf('\nstart pose: %d %d %d \n', rsInW.x, rsInW.y, rsInW.th);
            fprintf('goal pose in world frame: %d %d %d \n',xfa,yfa,thfa);
           
            rfInrs = pose.matToPoseVec((rsInW.bToA()^-1) * rfInW.bToA());

            
            %r = [xfa;yfa;thfa] - lpa;
            fprintf('goal traj: %d %d %d \n', rfInrs(1), rfInrs(2), rfInrs(3));
            
            curve = cubicSpiral.planTrajectory(rfInrs(1), rfInrs(2), rfInrs(3),sign);
            vmax=.2;
            planVelocities(curve, vmax);
            obj.follower.trajectory = curve;
            obj.follower.goalPose = [xfa,yfa,thfa];
           % obj.follower.startPose = obj.follower.lastPoser;
           % obj.follower.lastPoser = lpa;
           % obj.follower.lastPosef = lpd;
            
            obj.follower.feedForward(robot, obj.follower, true, 0.003,false);
            
            obj.lastStartPose = [xfa,yfa,thfa];
        end
        
        function moveRelDistance(obj,robot, dist, doControlPlotting)
            rpose = [0;0;0];
            tPause = 2;
            vmax = .25;
            amax = .75;
            sign = -1;
            tStepCtrl = trapezoidalStepReferenceControl(tPause, amax, vmax, dist, sign);

            time=0;
            timeArray = zeros(1,1);
            duration = tStepCtrl.getTrajectoryDuration(tStepCtrl);
            tdelay = 0.003;
            e_int = 0; %integral of error
            e_der = 0; %derivative of error
            e_pro = 0; %error 
            kp = 6; %proportional constant
            kd = 0.0125; %derivative constant
            ki = 0.00001;
            errorArray = zeros(1,1);
            feedback = false;

            i=2;
            leftStart = double(robot.encoders.LatestMessage.Left);
            leftEncoder = double(leftStart);

            %test tStepCtrl
            tic;
            while time < duration
                pause(0.001)
                time = toc; %elapsedTime between this loop and the previous one
                timeArray(i) = time;
                elapsedTime = time - timeArray(i-1);
                
                 
                [V,w] = tStepCtrl.computeControl(tStepCtrl,time);
                [vl,vr] = robotModel.VwTovlvr(V,w);

                sdelay = trapezoidalDistanceProfile((time-tdelay), amax, vmax, dist, sign);

                upid = 0;
                leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                signedDistance = double(double(leftEncoder - leftStart)/1000); %in m
                rpose = [signedDistance;
                        rpose(2);
                        rpose(3);];

                if(feedback)
                    errorArray(i) = sdelay - signedDistance;
                    x_i = sum(errorArray);
                    x_d = (double(errorArray(end) - double(errorArray(end-1))))/elapsedTime;
                    x_p = errorArray(end);
                    x_c = kp*x_p + ki*x_i + kd*x_d;
                    upid = x_c;
                    
%                     error = double(sdelay - signedDistance);
%                     e_pro = double(error/1000); %in m
%                     e_int = double(e_int) + error/1000; %in m*s
%                     e_der = double((error-errorArray(end))/1000/(time-timeArray(end))); %in m/s
%                     errorArray(i) = error;
%                     upid = kp*e_pro + ki * e_int + kd * e_der;
                end
                ureal = vl + upid;

                timeArray(i) = time;
                if  ureal < -0.3
                    ureal = -0.3;
                elseif ureal > 0.3
                    ureal = 0.3;
                end
                robot.sendVelocity(ureal, ureal);

            end
            disp(rpose);
            rInRs = pose(rpose);
            rsInw = pose(obj.follower.lastPoser);
            obj.follower.lastPoser = pose.matToPoseVec(rsInw.bToA()*rInRs.bToA());
            robot.sendVelocity(0,0);
            
        end
        
        function turnRelAngle(obj,robot,angle,doControlPlotting)
            rpose = [0;0;0];
            tPause = 2;
            wmax = pi()/3*1.4;
            awmax = pi()*1.6;
            sign = 1;
            delay = 0.0;
            tTurnCtrl = trapezoidalTurnReferenceControl(tPause, awmax, wmax, angle, sign);

            feedback = true;
            time=0;
            duration = tTurnCtrl.getTrajectoryDuration(tTurnCtrl);

            kp = 1e-3; %proportional constant
            kd = 0.68e-3; %derivative constant
            ki = 4.87e-4;

            thErrorArray = zeros(1,1);

            leftLast  = double(double(robot.encoders.LatestMessage.Left)/1000);
            leftNow   = double(leftLast); 
            rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
            rightNow  = double(rightLast);

            % distance the left and right wheels traveled in dt
            lds = 0;
            rds = 0;

            timeArray = zeros(1,1);
            wArray = zeros(1,1);
            thArray = zeros(1,1);
            threfArray = zeros(1,1);
            i=2;

            rotateTime = tic;
            while time < duration
                pause(0.003);
                timeArray(i) = toc(rotateTime);
                time = timeArray(i);
                dtime = time - timeArray(end-1);
                %elapsedTime = time - timeArray(i-1);  %elapsedTime between this loop and the previous one
                %time = time + elapsedTime;
               % elapsedTic = tic;

                dtime = time - timeArray(end-1);

                [V,w] = tTurnCtrl.computeControl(tTurnCtrl,time+delay);
                threfArray(i) = threfArray(i-1) + w*dtime;

                w_control = 0;

                % read encoder and update old reading
                %(of course, still in mm, divide by 1000)
                leftNow  = double(double(robot.encoders.LatestMessage.Left)/1000); 
                lds      = double(leftNow - leftLast);
                leftLast = double(leftNow);

                rightNow  = double(double(robot.encoders.LatestMessage.Right)/1000);
                rds       = double(rightNow - rightLast); 
                rightLast = double(rightNow);

                %get the real vl and vr
                real_vl = double(double(lds)/dtime); 
                real_vr = double(double(rds)/dtime);
                [V_real,w_real] = robotModel.vlvrToVw(real_vl,real_vr);
                dth = w_real*dtime;
                thArray(i) = thArray(i-1) + w_real*dth;
                rpose = [rpose(1);
                         rpose(2);
                         rpose(3) + dth];

                if(feedback)
                    %atan2?
                    thErrorArray(i) = atan2(sin(threfArray(i) - thArray(i)),...
                                            cos(threfArray(i) - thArray(i)));
                         
                    th_i = sum(thErrorArray);
                    th_d = (double(thErrorArray(end))-double(thErrorArray(end-1)))/dtime;
                    th_p = thErrorArray(end);
                    th_c = kp * th_p + ki * th_i + kd * th_d;

                    w_control = th_c;
                end

                [vl,vr] = robotModel.VwTovlvr(0,w+w_control);
                if(vl>0.3)
                   vl = 0.3;
                elseif vl<-0.3
                    vl = -0.3;
                end
                if vr >0.3
                   vr = 0.3;
                elseif vr < -0.3
                    vr = -0.3;
                end

                timeArray(i) = time;
                wArray(i) = w+w_control;
                i = i+1;

                if w+w_control>=0
                    robot.sendVelocity(vl,vr);
                end
            end
            
            robot.sendVelocity(0,0);
            %plot(timeArray, wArray);
            %rpose = [0;0;-pi];
            rInRs = pose(rpose);
            rsInw = pose(obj.follower.lastPoser);
            obj.follower.lastPoser = pose.matToPoseVec(rsInw.bToA()*rInRs.bToA());
            
        end
        
%         function executeTrajectory(obj,robot,x,y,th,sign) %"main method"
%             disp(nargin);
%             %make sure table is there.
%             
%             curve = cubicSpiral.planTrajectory(x,y,th,sign);
%             vmax=.3;
%             planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
%             %obj.follower.loadTrajectory(obj.follower,curve,obj.follower.lastPose);
%             obj.follower.trajectory = curve;
%             obj.follower.startPose = obj.follower.lastPoser;
%             %obj.follower.lastPoser = lpa;
%             %obj.follower.lastPosef = lpd;
%             obj.follower.feedForward(robot, obj.follower,true,0.003);
%            
%             %just plot the reference trajectory
%     %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
%         end
function pickDropObject(obj,robot,objPose)
            %first,from objPose, we get the aqPose
            aqDis = 0.45;% the distance we have between aqPose and pickupPose
            maxAngleDiff = pi/2;%the angle difference above which the robot turns
            PickInw = pose(objPose);
            PickInAq = pose([aqDis;0;0]);
            AqPoseInw = pose.matToPoseVec(PickInw.bToA()*PickInAq.aToB());
            aq_yaw = AqPoseInw(3);
            pose_yaw = obj.follower.lastPoser(3);
            
            angle_difference = atan2(sin(aq_yaw-pose_yaw),cos(aq_yaw-pose_yaw));
            if abs(angle_difference) > maxAngleDiff
                obj.turnRelAngle(obj,robot,angle_difference,1);
            end
            obj.executeTrajectorySE(obj,robot,AqPoseInw(1),AqPoseInw(2),AqPoseInw(3),1);
            lidarPickPose = obj.getLidarPickPose(obj,robot);
            obj.executeTrajectorySE(obj,robot,lidarPickPose(1),lidarPickPose(2),lidarPickPose(3),1);
            %fork up 
            
            %if the angle difference between pose and aqPose is big,
            % turn, then drive robot to aqPos, then start laser to find the
            % board, go to pick up pose and pick on board. back up and turn
            % towards drop location then drive to drop location. 
        end    
        
        %this function reads the lidar data to locate pickupPose
        function lidarPickPose = getLidarPickPose(obj,robot)
            disp('Starting Laser...');
            robot.startLaser();
            pause(5); % wait for laser data to come in
            rInw = pose(obj.follower.lastPoser);   
            rInSen = pose([0.11;0;0]);
            reading = transpose(double(robot.laser.LatestMessage.Ranges));
            image = rangeImage(reading,1,1); 
            disp('Laser values read');
            disp('Finding Best Midpoint...');
            [middle,bpose] = image.findObject(rInSen,rInw);
            fprintf('board pose found: %d %d %d', bpose(1),bpose(2),bpose(3));
            bInSen = pose(bpose);
            bInG = pose([0.038+0.1;0;0]);
            goalPoseInr = pose.matToPoseVec(rInSen.aToB() * bInSen.bToA() * bInG.aToB());
            goalInr = pose(goalPoseInr);                                                                                                                
            gPoseInw = pose.matToPoseVec(rInw.bToA()*goalInr.bToA());
            lidarPickPose = gPoseInw;            
        end
    end
    
end
