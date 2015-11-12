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
            obj.ctrl = controller(0.0001,0.0001,0.06);
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
            obj.follower.lastPoser = startPose;
            obj.follower.lastPosef = startPose;
            obj.SE.setInitPose(obj.SE, obj.follower.lastPoser);
        end
        function executeTrajectorySE(obj,robot,xfa,yfa,thfa,sign)
            %make sure table is there.
            lastPose = obj.follower.lastPoser;
            obj.SE.setInitPose(obj.SE, obj.follower.lastPoser);
            rth = lastPose(3);
            ry = lastPose(2);
            rx = lastPose(1);
            fprintf('\nstart pose: %d %d %d \n', rx, ry, rth);
            fprintf('goal pose in world frame: %d %d %d \n',xfa,yfa,thfa);
            %the matrix that transform from r frame to wolrd frame
            r_to_w_matrix = [cos(rth), -sin(rth), rx;
                             sin(rth),  cos(rth), ry;
                             0      ,  0      , 1];
            %the matrix that transform from destination frame to wolrd
            %frame (destination, xfa,yfa,thfa)
            d_to_w_matrix = [cos(thfa), -sin(thfa), xfa;
                             sin(thfa),  cos(thfa), xfa;
                             0      ,  0      , 1];
            %we need d_to_r_matrix = w_to_r * d_to_w;
            w_to_r_matrix = r_to_w_matrix^-1;
            d_to_r_matrix = w_to_r_matrix * d_to_w_matrix;
            result = w_to_r_matrix * [xfa;yfa;1];
            %arctan of (sin(th)/cos(th)), th is the angle of destination
            %pose in robot frame
            th = atan2(-d_to_r_matrix(1,2),d_to_r_matrix(1,1));
            
            %r = [xfa;yfa;thfa] - lpa;
            fprintf('goal traj: %d %d %d \n', result(1), result(2), th);
            
            curve = cubicSpiral.planTrajectory(result(1),result(2),th,sign);
            vmax=.25;
            planVelocities(curve, vmax);
            obj.follower.trajectory = curve;
           % obj.follower.startPose = obj.follower.lastPoser;
           % obj.follower.lastPoser = lpa;
           % obj.follower.lastPosef = lpd;
            
            
            obj.follower.feedForward(robot, obj.follower, true, 0.003);
            
            
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
