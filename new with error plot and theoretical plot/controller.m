classdef controller
    properties
        constMatrix;
        errorXArray;
        errorYArray;
        errorThArray;
        counter;
    end
    methods(Static = true)
        function obj = controller(kx, ky)
            obj.constMatrix = [kx 0; 0 ky];
            obj.counter=1;
            obj.errorXArray = zeros(1,1);
            obj.errorYArray = zeros(1,1);
            obj.errorThArray = zeros(1,1);
        end
        function [V_control, w_control] = velFeedback(obj, pose_robot, pose_ref, yaw_ref, robot_yaw) 
            %the poses 2x1 [x;y] are in world frame
            rotMatrix = [cos(robot_yaw) -sin(robot_yaw); sin(robot_yaw) cos(robot_yaw)];
            answer = obj.constMatrix*((rotMatrix^-1)*(pose_ref - pose_robot));
            answer = answer';
            V_control = answer(1);
            w_control = answer(2);
            
            %%************
            %obj.counter = counter;
            %errorInRobotFrame = ((rotMatrix^-1)*(pose_ref - pose_robot))';
            %obj.errorXArray(obj.counter) = errorInRobotFrame(1);
            %obj.errorYArray(obj.counter) = errorInRobotFrame(2);
            %obj.errorThArray(obj.counter) = yaw_ref - robot_yaw;
            %obj.counter = obj.counter+1;
           % errorXArray = obj.errorXArray;
           % errorYArray = obj.errorYArray;
           % errorThArray = obj.errorThArray;
           % counter = obj.counter;
            %%************
            
            %TODO: if pose is 1-dimentional, could change to return pid
            % velocity 
        end
        function eXArray = getErrorXArray(obj)%%************
            eXArray = obj.errorXArray;
        end
        function eYArray = getErrorYArray(obj)%%************
            eYArray = obj.errorYArray;
        end
        function eThArray = getErrorThArray(obj)%%************
            eThArray = obj.errorThArray;
        end
    end
end