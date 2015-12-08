classdef controller
    properties
        kp;
        kd;
        ki;
        lastState;
        started;
    end
    methods(Static = true)
        function obj = controller(kp,ki,kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.started = false;
            obj.lastState = [0;0;0];
        end
        function initialize(obj, startState)
            obj.lastState = startState;
            obj.started = false;
        end
        
        function [V_control, w_control] = velFeedback(obj,xErrorArray,yErrorArray,thErrorArray,dt)
            if (obj.started ~= false)
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
           
                V_control = x_c;%sqrt(x_c*y_c);
                w_control = y_c;%th_c;
            else
                V_control = 0;
                w_control = 0;
            end      
        end
    end
end