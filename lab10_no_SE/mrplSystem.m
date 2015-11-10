classdef mrplSystem < handle
    %HI this is main method 
    
    properties
        ctrl;
        follower;
    end
    
    methods(Static=true)
        function obj = mrplSystem()
            %obj.ctrl = controller(0.018,0.0000175,2e-2);
            obj.ctrl = controller(5e-3,1e-5,9e-3);
            obj.follower = trajectoryFollowerVerCubicSpiral(obj.ctrl);
        end
        function executeTrajectory(obj,robot,x,y,th,sign) %"main method"
            %make sure table is there.
            lastPose = obj.follower.lastPoser;
            rth = lastPose(3);
            ry = lastPose(2);
            rx = lastPose(1);
            
            s_to_f_matrix = [cos(rth), -sin(rth), rx;
                             sin(rth),  cos(rth), ry;
                             0      ,  0      , 1];
            f_to_s_matrix = s_to_f_matrix^-1;
            result = f_to_s_matrix * [x;y;1];
            th = atan2(sin(th-rth),cos(th-rth));
            disp([result(1),result(2),th]);
            curve = cubicSpiral.planTrajectory(result(1),result(2),th,sign);
            vmax=.25;
            planVelocities(curve, vmax); %fills in the v,w,vl,vr arrays
            %obj.follower.loadTrajectory(obj.follower,curve,obj.follower.lastPose);
            obj.follower.trajectory = curve;
            obj.follower.startPose = obj.follower.lastPoser;
            obj.follower.feedForward(robot, obj.follower,true,0.501);
           
            
            %just plot the reference trajectory
    %        plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
        end
    end
    
end

