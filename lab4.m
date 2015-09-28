%% dafdafsd

robot = neato('yotta');

%%

robot.close();
clear all;
%% warm up 3.1

feedback = true; %boolean deciding to add feedback factor

timeArray = zeros(1,1);
distArray = zeros(1,1); % create 1x1 matrix with value 0
urealArray = zeros(1,1);
sDelayArray = zeros(1,1);
errorArray = zeros(1,1);

leftStart = double(robot.encoders.LatestMessage.Left);
leftEncoder = double(leftStart); % in "mm"

signedDistance =0;
arrayIndex = 1; %index starts with 1

time = 0;

vmax = 0.25;
amax = 3*0.25;
dist = 1;
sign = 1; %change this to a negative number if the goal is behind the robot
tf = (dist + ((vmax^2)/amax))/vmax;

tdelay = -0.003;

%PID control constants
kp = 6; %proportional constant
kd = 6; %derivative constant
ki = 2.3; %integral constant
e_int = 0; %integral of error
e_der = 0; %derivative of error
e_pro = 0; %error 

while time < (tf)
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    
    uref = trapezoidalVelocityProfile(time, amax, vmax, dist, sign); %reference velocity
    
    sdelay = trapezoidalDistanceProfile((time-tdelay), amax, vmax, dist, sign);
    
    leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
    signedDistance = double(double(leftEncoder - leftStart)/1000); %in m
    
    upid = 0;
    
    if(feedback)
        error = double(sdelay - signedDistance);
        e_pro = double(error/1000); %in m
        e_int = double(e_int) + error/1000; %in m*s
        e_der = double((error-errorArray(end))/1000/(time-timeArray(end))); %in m/s
        upid = kp*e_pro + ki*e_int + kd*e_der;
    end
    ureal = uref + upid;
    
    if(ureal<=0)
        ureal= 0;
    end
    if(ureal>3)
        ureal=3;
    end
    
    robot.sendVelocity(ureal, ureal);
    
    timeArray(arrayIndex) = time; %total elapsed time so far
    distArray(arrayIndex) = signedDistance; %total travelled distance so far
 %   srefArray(arrayIndex) = sref; %plot uref? sref
    urealArray(arrayIndex) = ureal;
    sDelayArray(arrayIndex) = sdelay;
    arrayIndex = arrayIndex + 1;
end

max(distArray)

robot.sendVelocity(0.0, 0.0);
pause(1);

figure(1);
plot(timeArray, distArray, timeArray, sDelayArray);
figure(2);
plot(timeArray, distArray - sDelayArray);
