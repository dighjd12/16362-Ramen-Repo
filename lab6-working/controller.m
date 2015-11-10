classdef controller
    properties
<<<<<<< Updated upstream
        kx;
        ky;
        convertMatrix;
        lastState;
        started;
    end
    methods(Static = true)
        function obj = controller(kx,ky)
            obj.kx = kx;
            obj.ky = ky;
            obj.convertMatrix = [kx,0;0,ky];
            obj.started = false;
            %last state?
=======
        kp;
        kd;
        ki;
    end
    methods(Static = true)
        function obj = controller(kp,ki,kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
>>>>>>> Stashed changes
        end
        function [V_control, w_control] = velFeedback(obj,xErrorArray,yErrorArray,thErrorArray,dt)
           %integrate parts
           x_i = sum(xErrorArray);
           y_i = sum(yErrorArray);
           th_i = sum(thErrorArray);
           %derivative parts
           x_d = (double(xErrorArray(end))-double(xErrorArray(end-1)))/dt;
           y_d = (double(yErrorArray(end))-double(yErrorArray(end-1)))/dt;
           th_d = (double(thErrorArray(end))-double(thErrorArray(end-1)))/dt;
           
           x_p = xErrorArray(end);
           y_p = yErrorArray(end);
           th_p = thErrorArray(end);
           
           x_c = obj.kp * x_p + obj.ki * x_i + obj.kd * x_d;
           y_c = obj.kp * y_p + obj.ki * y_i + obj.kd * y_d;
           th_c = obj.kp * th_p + obj.ki * th_i + obj.kd * th_d;
           
           V_control = x_c+y_c+th_c;
           w_control = x_c+y_c+th_c;
            
        end
        function initialize(obj, startState)
           obj.lastState = startState; % 3x1 pose
           obj.started = false; %initialize clock,derivative...??
        end
    end
end