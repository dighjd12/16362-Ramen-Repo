classdef controller
    properties
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
        end
        function [V_control, w_control] = velFeedback(obj,errorInRobotFrame) 
            control = obj.convertMatrix*errorInRobotFrame;   
            V_control = control(1);
            w_control = control(2);
        end
        function initialize(obj, startState)
           obj.lastState = startState; % 3x1 pose
           obj.started = false; %initialize clock,derivative...??
        end
    end
end