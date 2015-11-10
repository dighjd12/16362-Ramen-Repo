classdef mrplSystem
    %HI this is main method 
    
    properties
    end
    
    methods(Static=true)
        function executeTrajectory(robot,x,y,th,sign,tryNum) %"main method"
            ctrl = controller(1,0,0);
            %make sure table is there.
            curve = cubicSpiral.planTrajectory(x,y,th,sign);
            vmax=.25;
            planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
<<<<<<< Updated upstream
            follower = trajectoryFollowerVerCubicSpiral(ctrl);
            
            follower.loadTrajectory(follower, curve, follower.lastPose)
            
            follower.feedForward(robot, follower,false,-0.003,tryNum);
=======
            follower = trajectoryFollowerVerCubicSpiral(ctrl, curve);
            follower.feedForward(robot, follower,true,-0.0007,tryNum);
>>>>>>> Stashed changes
            
            %just plot the reference trajectory
    %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
        end
    end
    
end

