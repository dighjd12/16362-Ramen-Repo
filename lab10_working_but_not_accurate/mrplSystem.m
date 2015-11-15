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
            obj.ctrl = controller(0.003,0,0);
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
            
            obj.follower.feedForward(robot, obj.follower, false, 0.05,false);
            
            obj.lastStartPose = [xfa,yfa,thfa];
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
