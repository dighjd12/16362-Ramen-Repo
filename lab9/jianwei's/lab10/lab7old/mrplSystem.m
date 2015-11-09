classdef mrplSystem
    %HI this is main method 
    
    properties
        ctrl;
        follower;
    end
    
    methods(Static=true)
        function obj = mrplSystem()
            %obj.ctrl = controller(0.018,0.0000175,2e-2);
            obj.ctrl = controller(0.05,0.00005,0.05);
            obj.follower = trajectoryFollowerVerCubicSpiral(obj.ctrl);
        end
        function executeTrajectory(obj,robot,x,y,th,sign) %"main method"
            %make sure table is there.
            curve = cubicSpiral.planTrajectory(x,y,th,sign);
            vmax=.25;
            planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
            obj.follower.loadTrajectory(obj.follower,curve,obj.follower.lastPose);
            obj.follower.feedForward(robot, obj.follower,true,0.501);
           
            
            %just plot the reference trajectory
    %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
        end
    end
    
end

