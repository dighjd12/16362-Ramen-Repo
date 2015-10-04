classdef controller
    properties
        constMatrix;
    end
    methods(Static = true)
        function obj = controller(kx, ky)
            obj.constMatrix = [kx 0; 0 ky];
        end
        function [V_control, w_control] = velFeedback(pose_robot, pose_ref, robot_yaw) 
            %the poses 2x1 [x;y] are in world frame
            rotMatrix = [cos(robot_yaw) -sin(robot_yaw); sin(robot_yaw) cos(robot_yaw)];
            answer = obj.constMatrix*((rotMatrix^-1)*(pose_ref - pose_robot));
            answer = answer';
            V_control = answer(1);
            w_control = answer(2);
             
            %TODO: if pose is 1-dimentional, could change to return pid
            % velocity 
        end
    end
end