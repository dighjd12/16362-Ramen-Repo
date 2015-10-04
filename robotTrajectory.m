classdef robotTrajectory
    properties
        numSamples;
        time;
        v;
        w;
        pose; %[x; y]
        dist;
        referenceControl;
    end
    methods(Static = true)
        function obj = robotTrajectory(ti, tf, dt, s_o, p_o, referenceControl)
             %??
            obj.numSamples = (tf-ti)./dt;
            %tf should be within the duration of referenceControl
            obj.time(1) = ti;
            obj.dist(1) = s_o;
            obj.pose(1) = p_o; % 2x1 [x;y] vector
            %original 3x3 homogeneous transform wrt world frame whatever that is
            % [  c(theta) -s(theta) x
            %    s(theta)  c(theta) y
            %     0   0 1 ]
            obj.referenceControl = referenceControl;
            %fig_ref = figure8ReferenceControl(0.5,0.4,0.5);

            for i=1:n-1
                [V_i,w_i] = referenceControl.computeControl(fig_ref, obj.time(i));
                obj.v(i) = V_i;
                obj.w(i) = w_i;
                obj.dist(i+1) = obj.dist(i) + obj.v(i)*dt;
                dth = obj.w(i)*dt;
                ds = obj.v(i)*dt;
               % T_i = [cos(dth) -sin(dth) ds*cos(dth); sin(dth) cos(dth) ds*sin(dth); 0 0 1];
                obj.pose(i+1) = obj.pose(i) + [ds*cos(dth); ds*sin(dth)];%obj.pose(i)*T_i;
                obj.time(i+1) = t(i)+dt;
            end
            obj.v(n) = 0;
            obj.w(n) = 0;
        
        end
        function pose_t = getPoseAtTime(obj,t)
             pose_t = interp1(obj.time,obj.pose,t); %should work..
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