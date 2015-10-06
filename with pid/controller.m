classdef controller
    properties
        kp;
        ki;
        kd;
        %constMatrix;
        %errorXArray;
        %errorYArray;
        %errorThArray;
        %counter;
    end
    methods(Static = true)
        function obj = controller(kp, ki, kd)
            %obj.constMatrix = [kx 0; 0 ky];
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
        end
        function [V_control, w_control] = velFeedback(obj,xErrorArray,yErrorArray,thErrorArray,dt) 
            %the poses 2x1 [x;y] are in world frame
            x_i = sum(xErrorArray);
            y_i = sum(yErrorArray);
            th_i = sum(thErrorArray);
            if length(xErrorArray) == 1;
                x_d = double(xErrorArray(end)/double(dt));
            else
                x_d = double((xErrorArray(end) - xErrorArray(end-1))/double(dt));
            end
            if length(xErrorArray) == 1;
                y_d = double(yErrorArray(end)/double(dt));
            else
                y_d = double((yErrorArray(end) - yErrorArray(end-1))/double(dt));
            end
            if length(thErrorArray) == 1;
                th_d = double(thErrorArray(end)/double(dt));
            else
                th_d = double((thErrorArray(end) - thErrorArray(end-1))/double(dt));
            end
            x_p = xErrorArray(end);
            y_p = yErrorArray(end);
            th_p = thErrorArray(end);
            x_control = (x_i*obj.ki + x_d*obj.kd + x_p * obj.kp);
            y_control = (y_i*obj.ki + y_d*obj.kd + y_p * obj.kp);
            th_control = th_i*obj.ki + th_d*obj.kd + th_p * obj.kp;
            V_control = sqrt(x_control^2 + y_control^2);
            w_control =th_control;
            %disp(V_control);
            %disp(w_control);
            
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