classdef trapezoidalStepReferenceControl
    properties
        tramp;
        amax;
        tPause;
        vmax;
        dist;
        signFactor;
        tf;
    end
    methods(Static = true)
        function obj = trapezoidalStepReferenceControl(tPause, amax, vmax, dist, sign)
            obj.tPause = tPause;
            obj.vmax = vmax;
            obj.amax = amax;
            obj.dist = dist;
            obj.tramp = vmax/amax;
            obj.tf = (dist + (vmax^2)/amax)/vmax;
            obj.signFactor = 1;
            if (sign<0)
                obj.signFactor = -1;
            end
        end
        function [V,w] = computeControl(obj,timeNow)
        start_pause = timeNow <= obj.tPause && timeNow >= 0;
        end_pause = timeNow>obj.tf && timeNow < (obj.tf+obj.tPause);
        w = 0;
        if (start_pause || end_pause)
            V = 0;
            
        else
            timeNow = timeNow - obj.tPause;
            V =0;

            if(timeNow<obj.tramp)
                V = obj.amax*t;
            end

            if(obj.tramp<timeNow && timeNow<(obj.tf-obj.tramp))
                V = obj.vmax;
            end

            if((obj.tf-timeNow)<obj.tramp)
                V = (obj.amax*(obj.tf-timeNow));
            end

            V = V*obj.signFactor;

            if(timeNow<0 || timeNow>obj.tf)
                V = 0;
            end
            
        end
        end
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = double(obj.tPause*2 + obj.tf);
        end
    end
end