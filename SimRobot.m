% Class:        SimRobot.m
% Description:  tracks position of robot based on encoder readings (either
%               real or simulated)
% Last Edit:    10/19/2015
classdef SimRobot
    properties
        % in world frame
        x_pos;
        y_pos;
        % velocities
        left_vel;
        right_vel;
        % time of last encoder reading / last time reading
        last_left_encoder;
        last_right_encoder;
        last_left_time;
        last_right_time;
        
        % constants
        WHEEL_BASE = 0.23495;
    end
    methods(Static = true)
        % Function:     SimRobot
        % Description:  initialize object.  Currently just zeros values
        % Takes:        
        %   none
        % Returns:      
        %   obj           - updated SimRobot object
        % Last Edit:    10/19/2015
        function obj = simRobot()
            obj.x_pos              = 0;
            obj.y_pos              = 0;
            obj.left_vel           = 0;
            obj.right_vel          = 0;
            obj.last_left_encoder  = 0;
            obj.last_right_encoder = 0;
            obj.last_left_time     = 0;
            obj.last_right_time    = 0; 
        end
        
        % Function:     updatePos
        % Description:  update position of robot in world frame based on
        %               encoder readings
        % Takes:        
        %   obj           - SimRobot object
        %   left_encoder  - left encoder reading
        %   right_encoder - right encoder reading
        % Returns:      
        %   obj           - updated SimRobot object
        % Last Edit:    10/19/2015
        function obj = updatePos(obj,left_encoder, right_encoder)
            % integrate time
            distance = (left_encoder + right_encoder) / 2.0;
            theta    = (left_encoder - right_encoder) / obj.WHEEL_BASE;
            
            obj.x_pos = distance * sin(theta);
            obj.y_pos = distance * cos(theta);
            obj.heading = theta  * (180/pi);
            
        end 
        % Function:     updateSpeed
        % Description:  update speed of robot based on encoder readings and
        %               time stamps
        % Takes:        
        %   obj           - SimRobot object
        %   left_encoder  - left encoder reading
        %   right_encoder - right encoder reading
        %   left_time     - time of left encoder reading
        %   right_time    - time of right encoder reading
        % Returns:      
        %   obj           - updated SimRobot object
        % Last Edit:    10/19/2015
        function obj = updateSpeed(obj, left_encoder, right_encoder, left_time, right_time)
            % calculate velocities
            obj.left_vel  = (left_encoder - obj.last_left_encoder)   / (left_time - obj.last_left_time);
            obj.right_vel = (right_encoder - obj.last_right_encoder) / (right_time - obj.last_right_time);
            
            % update last encoder readings
            obj.last_left_encoder  = left_encoder;
            obj.last_right_encoder = right_encoder;
            
            % update last time readings
            obj.last_left_time  = left_time;
            obj.last_right_time = right_time;
        end
    end
end
