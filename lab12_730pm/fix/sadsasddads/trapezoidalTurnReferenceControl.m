classdef trapezoidalTurnReferenceControl
    properties
        tramp;
        amax;
        tPause;
        wmax;
        angle;
        signFactor;
        tf;
    end
    methods(Static = true)
        function obj = trapezoidalTurnReferenceControl(tPause, amax, wmax, angle, sign)
            obj.tPause = tPause;
            obj.wmax = wmax;
            obj.amax = amax;
            obj.angle = angle;
            obj.tramp = wmax/amax;
            obj.tf = (angle + (wmax^2)/amax)/wmax;
            obj.signFactor = 1;
            if (sign<0)
                obj.signFactor = -1;
            end
        end
        function [V,w] = computeControl(obj,timeNow)
        start_pause = timeNow <= obj.tPause;
        end_pause = timeNow>(obj.tf+obj.tPause);
        w = 0;
        V = 0;
        if (start_pause || end_pause)
            V = 0;
            w = 0;
        else
            timeNow = timeNow - obj.tPause;

            if(timeNow<obj.tramp)
                w = obj.amax*timeNow;
            end

            if(obj.tramp<timeNow && timeNow<(obj.tf-obj.tramp))
                w = obj.wmax;
            end

            if((obj.tf-timeNow)<obj.tramp)
                w = (obj.amax*(obj.tf-timeNow));
            end

            w = w*obj.signFactor;
            
        end
        
        end
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = double(obj.tPause*2 + obj.tf);
        end
    end
end