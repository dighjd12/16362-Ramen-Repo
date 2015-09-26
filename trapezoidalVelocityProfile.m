function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sign)

%vmax = .25m/s, amax = 3*.25m/s^2, tramp = vamx/amax, dist = 1
%sign is -1 if the goal is behind the robot

tramp = vmax/amax;
tf = (dist + (vmax^2)/amax)/vmax;
signFactor = 1;

if(sign<0)
    signFactor = -1;
end

uref =0;

if(t<tramp)
    uref = amax*t;
end

if(tramp<t && t<(tf-tramp))
    uref = vmax;
end

if((tf-t)<tramp)
    uref = (amax*(tf-t));
end

uref = uref*signFactor;

if(t<0 || t>tf)
    uref = 0;
end

end


