classdef controller
    properties
        kx;
        ky;
        convertMatrix;
    end
    methods(Static = true)
        function obj = controller(kx,ky)
            obj.kx = kx;
            obj.ky = ky;
            obj.convertMatrix = [kx,0;0,ky];
        end
        function [V_control, w_control] = velFeedback(obj,errorInRobotFrame) 
            control = obj.convertMatrix*errorInRobotFrame;   
            V_control = control(1);
            w_control = control(2);
        end
    end
end