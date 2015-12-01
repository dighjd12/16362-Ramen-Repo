classdef mrplSystem < handle
    %HI this is main method 
    
    properties
        ctrl;
        follower;
        SE;
        startPose;
    end
    
    methods(Static=true)
        function obj = mrplSystem()
            obj.ctrl = controller(0.05,0.00001,0.05);
            lines_p1 = [[0;0], [0;4]];
            lines_p2 = [[4;0], [0;0]];
            walls = [[4.0; 0.0], [0.0; 0.0], [0.0; 4.0]];
            mrplSystem.addStateEstimator(obj, 10, lines_p1, lines_p2, walls);
            obj.follower = trajectoryFollowerVerCubicSpiral(obj.ctrl, obj.SE);
            
        end
        function addStateEstimator(obj, lidarSkip, lines_p1, lines_p2, worldLineArray)
            gain = 0.01;
            errThresh = 0.001;
            gradThresh = 0.0005;
            lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
            obj.SE = stateEstimator(lmLocalizer, worldLineArray, lidarSkip);
        end
        function setInitialPose(obj, startPose)
            obj.follower.robotStart = startPose;
        end
        function executeTrajectorySE(obj,robot,xfa,yfa,thfa,sign)
            %make sure table is there.
            
            lastPose = obj.follower.robotStart;
            
            rsInW = pose(lastPose);
            rfInW = pose(xfa,yfa,thfa);
            
            rfInRs = pose.matToPoseVec((rsInW.bToA()^-1) * rfInW.bToA());
            disp(rfInRs);
%             rth = lastPose(3);
%             ry = lastPose(2);
%             rx = lastPose(1);
%             fprintf('start pose: %d %d %d \n', rx, ry, rth);
%             
%             r_to_w_matrix = [cos(rth), -sin(rth), rx;
%                              sin(rth),  cos(rth), ry;
%                              0      ,  0      , 1];
%             w_to_r_matrix = r_to_w_matrix^-1;
%             result = w_to_r_matrix * [xfa;yfa;1];
%             th = atan2(sin(thfa-rth),cos(thfa-rth));
%             
%             %r = [xfa;yfa;thfa] - lpa;
%             fprintf('goal traj: %d %d %d \n', result(1), result(2), th);
            
            curve = cubicSpiral.planTrajectory(rfInRs(1),rfInRs(2),rfInRs(3),sign);
            vmax=.25;
            planVelocities(curve, vmax);
            obj.follower.trajectory = curve;
            obj.follower.startPose = obj.follower.lastPoser;
           % obj.follower.lastPoser = lpa;
           % obj.follower.lastPosef = lpd;
            
            obj.follower.feedForward(robot, obj.follower, true, 0.25);
            
            obj.follower.robotStart = [xfa;yfa;thfa];
        end
        function executeTrajectory(obj,robot,x,y,th,sign) %"main method"
            disp(nargin);
            %make sure table is there.
            
            curve = cubicSpiral.planTrajectory(x,y,th,sign);
            vmax=.25;
            planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
            %obj.follower.loadTrajectory(obj.follower,curve,obj.follower.lastPose);
            obj.follower.trajectory = curve;
            obj.follower.startPose = obj.follower.lastPoser;
            %obj.follower.lastPoser = lpa;
            %obj.follower.lastPosef = lpd;
            obj.follower.feedForward(robot, obj.follower,true,0.501);
           
            %just plot the reference trajectory
    %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
        end
    end
    
end
