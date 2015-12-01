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
            obj.ctrl = controller(8e-3,1e-2,-3e-3);
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
            vmax=.25;
            planVelocities(curve, vmax);
            obj.follower.trajectory = curve;
            obj.follower.goalPose = [xfa,yfa,thfa];
           % obj.follower.startPose = obj.follower.lastPoser;
           % obj.follower.lastPoser = lpa;
           % obj.follower.lastPosef = lpd;
            
            obj.follower.feedForward(robot, obj.follower, true, 0.003,true);
            
            obj.lastStartPose = [xfa,yfa,thfa];
        end
        
        function moveRelDistance(obj,robot, dist, doControlPlotting)
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
            feedback = true;

            i=2;
            leftStart = double(robot.encoders.LatestMessage.Left);
            leftEncoder = double(leftStart);

            %test tStepCtrl
            elapsedTic = tic;
            while time < duration
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                time = time + elapsedTime;
                elapsedTic = tic;
                
                
                
                [V,w] = tStepCtrl.computeControl(tStepCtrl,time);
                [vl,vr] = robotModel.VwTovlvr(robotModel,V,w);

                sdelay = trapezoidalDistanceProfile((time-tdelay), amax, vmax, dist, sign);

                upid = 0;
                leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
                signedDistance = double(double(leftEncoder - leftStart)/1000); %in m

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
            robot.sendVelocity(0,0);
            
        end
        
        function turnRelAngle(obj,robot,angle,doControlPlotting)
            tPause = 2;
            wmax = pi()/2;
            awmax = 3*pi()/2;
            sign = 1;
            delay = 0.005;%0.003;
            tTurnCtrl = trapezoidalTurnReferenceControl(tPause, awmax, wmax, angle, sign);

            feedback = true;
            time=0;
            duration = tTurnCtrl.getTrajectoryDuration(tTurnCtrl);

            kp = 0; %proportional constant
            kd = 0; %derivative constant
            ki = 0;

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

            elapsedTic = tic;
            while time < duration+delay
                elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
                time = time + elapsedTime;
                elapsedTic = tic;

                dtime = time - timeArray(end);

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
                [V_real,w_real] = robotModel.vlvrToVw(robotModel,real_vl,real_vr);
                dth = w_real*dtime;
                thArray(i) = thArray(i-1) + w_real*dth;

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

                [vl,vr] = robotModel.VwTovlvr(robotModel,0,w+w_control);
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
                wArray(i) = w;
                i = i+1;

                robot.sendVelocity(vl,vr);
            end
            
            robot.sendVelocity(0,0);
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
    end
    
end