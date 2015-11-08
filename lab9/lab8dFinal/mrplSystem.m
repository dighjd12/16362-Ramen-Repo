classdef mrplSystem < handle
    %HI this is main method 
    
    properties
        ctrl;
        follower;
        SE;
    end
    
    methods(Static=true)
        function obj = mrplSystem()
            %obj.ctrl = controller(0.018,0.0000175,2e-2);
            %obj.ctrl = controller(0.05,0.00005,0.05);
            obj.ctrl = controller(0.05,0.00001,0.05);
            obj.follower = trajectoryFollowerVerCubicSpiral(obj.ctrl);
        end
        function addStateEstimator(obj, lidarSkip, lines_p1, lines_p2, worldLineArray)
            obj.SE = %%
        end
        function [lpb,lpc] = executeTrajectorySE(obj,robot,xfa,yfa,thfa,sign,lpa,lpd)
            %make sure table is there.
            curve = cubicSpiral.planTrajectory(xfa,yfa,thfa,sign);
            vmax=.25;
            planVelocities(curve, vmax);
            
            obj.follower.trajectory = curve;
            obj.follower.startPose = obj.follower.lastPoser;
            obj.follower.lastPoser = lpa;
            obj.follower.lastPosef = lpd;
            
            [lpb,lpc] = obj.follower.feedForwardSE(robot, obj.follower,0.501);
            
        end
        function [lpb,lpc] = executeTrajectory(obj,robot,x,y,th,sign,lpa,lpd) %"main method"
            disp(nargin);
            %make sure table is there.
            curve = cubicSpiral.planTrajectory(x,y,th,sign);
            vmax=.25;
            planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
            %obj.follower.loadTrajectory(obj.follower,curve,obj.follower.lastPose);
            obj.follower.trajectory = curve;
            obj.follower.startPose = obj.follower.lastPoser;
            obj.follower.lastPoser = lpa;
            obj.follower.lastPosef = lpd;
            [lpb,lpc] = obj.follower.feedForward(robot, obj.follower,true,0.501);
           
            
            %just plot the reference trajectory
    %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
        end
    end
    
end

