classdef robotModel
    properties
        W = 241.3;
        W2 = 120.65;
    end
    methods(Static = true)
        function [V,w] = vlvrToVw(obj,vl,vr)
            %get linear and angular velocities from vl and vr.
            V = (double(vl)+double(vr))/2;
            w = (double(vr) - double(vl))/double(obj.W);
        end
        function [vl,vr] = VwTovlvr(obj,V,w)
            %get vl and vr from linear and angular velocities
            vr = ((w * obj.W)+(V*2))/ 2;
            vl = ((V*2)-(w * obj.W)) /2;
        end
    end
end