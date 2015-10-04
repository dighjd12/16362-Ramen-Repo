classdef robotTrajectory
    properties
        numSamples;
        time;
        v;
        w;
        x;
        y;
        dist;
        referenceControl;
    end
    methods(Static = true)
        function obj = robotTrajectory(ti, tf, dt, s_o, p_o, referenceControl)
            obj.numSamples = round((tf-ti)/dt);
            %tf should be within the duration of referenceControl
           
            p_o = p_o';
            obj.x(1) = p_o(1);
            obj.y(1) = p_o(2);
            obj.time(1) = ti;
            obj.dist(1) = s_o;
            
            obj.referenceControl = referenceControl;
            
            for i=1:obj.numSamples-1
                [V_i,w_i] = referenceControl.computeControl(referenceControl, obj.time(i));
                obj.v(i) = V_i;
                obj.w(i) = w_i;
                obj.dist(i+1) = obj.dist(i) + obj.v(i)*dt;
                dth = obj.w(i)*dt;
                ds = obj.v(i)*dt;
                obj.x(i+1) = obj.x(i) + ds*cos(dth);
                obj.y(i+1) = obj.y(i) + ds*sin(dth);
                obj.time(i+1) = obj.time(i)+dt;
            end
            obj.v(obj.numSamples) = 0;
            obj.w(obj.numSamples) = 0;
        
        end
        function pose_t = getPoseAtTime(obj,t)
            x_t = interp1(obj.time,obj.x,t);
            y_t = interp1(obj.time,obj.y,t);
            pose_t = [x_t; y_t]; 
        end
        function vel_t = getVelocityAtTime(obj,t)
            vel_t = interp1(obj.time,obj.v,t);
        end
        function dist_t = getDistanceAtTime(obj,t)
            dist_t = interp1(obj.time,obj.dist,t);
        end
        function w_t = getOmegaAtTime(obj,t)
            w_t = interp1(obj.time, obj.w, t);
        end
    end
end