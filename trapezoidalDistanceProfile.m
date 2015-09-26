function sref = trapezoidalDistanceProfile(t, amax, vmax, dist, sign)

%vmax = .25m/s, amax = 3*.25m/s^2, tramp = vamx/amax, dist = 1
%sign is -1 if the goal is behind the robot

tramp = vmax/amax;
tf = (dist + ((vmax^2)/amax))/vmax;
signFactor = 1;

if(sign<0)
    signFactor = -1;
end

sref =0;

if(t<tramp) %at the first ramp
    sref = amax*t^2/2; %integral of amax*t, assuming t_initial = 0;
end

if(tramp<t && t<(tf-tramp)) %in between the ramps
    sref = vmax*(t-tramp) + amax*tramp^2/2;
end

if((tf-t)<tramp) %at the second ramp
    %tbefore = t - tramp - (tf-2*tramp); % t before the second ramp
    %(amax*tf*t - amax*t^2/2) - (amax*tf*tbefore - amax/2*tbefore) +
    sref = (amax*tramp^2/2 - amax*(tf-t)^2/2) + vmax*(tf-2*tramp) + amax*tramp^2/2;
end

sref = sref*signFactor;

if(t<0)
    sref = 0;
end

end