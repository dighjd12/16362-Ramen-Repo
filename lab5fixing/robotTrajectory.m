classdef robotTrajectory
    properties
        numSamples;
        time;
        v;
        w;
        x;
        y;
        th;
        dist;
        referenceControl;
    end
    methods(Static = true)
        function obj = robotTrajectory(ti, tf, dt, s_o, p_o, referenceControl)
            % get every [x y th] V w on the traj defined by
            % referenceControl from ti to tf. time step is dt. the initial
            % pose and distance traveled are s_o and p_o
            
            %length of the array 
            obj.numSamples = round((tf-ti)/dt);
            %tf should be within the duration of referenceControl
           
            p_o = p_o'; %make [ x y th] into [x
                        %                     y
                        %                     th], transpose
            %preallocate x, y, th array
            obj.x = zeros(1, obj.numSamples);
            obj.y = zeros(1, obj.numSamples);
            obj.v = zeros(1,obj.numSamples);
            obj.w = zeros(1,obj.numSamples);
            obj.th = zeros(1, obj.numSamples);
            obj.time = zeros(1, obj.numSamples);
            obj.dist = zeros(1, obj.numSamples);
            %init the first values
            obj.x(1) = p_o(1);
            obj.y(1) = p_o(2);
            obj.th(1) = p_o(3);
            obj.time(1) = ti;
            obj.dist(1) = s_o;
            %init the reference shape we need to get
            obj.referenceControl = referenceControl;
            %iterate by dt from ti to tf get [x,y,th] V w at each moment
            for i=2:obj.numSamples
                %get the V and w at a moment
                obj.time(i) = obj.time(i-1)+dt;
                [V_i,w_i] = referenceControl.computeControl(referenceControl, obj.time(i));
                obj.v(i) = V_i;
                obj.w(i) = w_i;
                %distance and angle changed from last moment
                dth = double(w_i*dt);
                ds = double(V_i * dt);
                %updating array 
                obj.th(i) = obj.th(i-1) + dth;
                th_l = obj.th(i-1); %last th angle to get x and y updated
                disp_y = ds * sin(th_l);
                disp_x = ds * cos(th_l);
                obj.x(i) = obj.x(i-1) + disp_x;
                obj.y(i) = obj.y(i-1) + disp_y;
            end
        end
        function pose_t = getPoseAtTime(obj,t)
            x_t = interp1(obj.time,obj.x,t);
            y_t = interp1(obj.time,obj.y,t);
            th_t = interp1(obj.time,obj.th,t);
            pose_t = [x_t; y_t;th_t]; 
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