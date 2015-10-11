classdef figure8ReferenceControl
    properties
        tPause;
        Ks;
        Kv;
        tf;
    end
    methods(Static = true)
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
         % Construct a figure 8 trajectory. It will not start until
         % tPause has elapsed and it will stay at zero for tPause
         % afterwards. Kv scales velocity up when > 1 and Ks scales
         % the size of the curve itself down. 0.4 is a good value
         % for both.
        obj.tPause = tPause;
        obj.Ks = Ks;
        obj.Kv = Kv;
        obj.tf = double(12.565 / Kv * Ks);
        end
        function [V,w] = computeControl(obj,timeNow)
         % Return the linear and angular velocity that the robot
         % should be executing at time timeNow. Any zero velocity
         % pauses specified in the constructor are implemented here
         % too.
        start_pause = timeNow <= obj.tPause;
        end_pause = timeNow>(obj.tf+obj.tPause);
        if (start_pause || end_pause)
            V = 0;
            w = 0;
        else
            t = timeNow-obj.tPause;
            ks = obj.Ks;
            kv = obj.Kv;
            vr = double((0.3*kv + 0.14125*(kv/ks) * sin((t.*kv)./(2*ks))));
            vl = double((0.3*kv - 0.14125*(kv/ks) * sin((t.*kv)./(2*ks))));
            [V, w] = robotModel.vlvrToVw(robotModel,vl,vr);
        end
        end
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = double(obj.tPause*2 + obj.tf);
        end
    end
end