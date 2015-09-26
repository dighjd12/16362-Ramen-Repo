%% dafdafsd

robot = neato('exa');

%%

robot.close();
clear all;
%% warm up 3.1

%int_dist = 20;
timeArray = zeros(1,1);
distArray = zeros(1,1); % create 1x1 matrix with value 0
srefArray = zeros(1,1);


leftStart = robot.encoders.LatestMessage.Left;
leftEncoder = leftStart; % in "mm"

signedDistance =0;
arrayIndex = 1; %index starts with 1

time = 0;

vmax = 0.25;
amax = 3*0.25;
dist = 1;
sign = 1; %change this to a negative number if the goal is behind the robot
tf = (dist + ((vmax^2)/amax))/vmax;
tdelay = 2;

while time < tf
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    
    uref = trapezoidalVelocityProfile(time, amax, vmax, dist, sign); %reference velocity
    sref = trapezoidalDistanceProfile(time, amax, vmax, dist, sign); %reference distance
   
    udelay = trapezoidalVelocityProfile(time-tdelay, amax, vmax, dist, sign);
    sdelay = trapezoidalDistanceProfile(time-tdelay, amax, vmax, dist, sign);
    
    robot.sendVelocity(udelay, udelay);
    
    leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
    signedDistance = (leftEncoder - leftStart)/1000; %in m
    
    timeArray(arrayIndex) = time; %total elapsed time so far
    distArray(arrayIndex) = signedDistance; %total travelled distance so far
    srefArray(arrayIndex) = sref; %plot uref? sref
    arrayIndex = arrayIndex + 1;
end

robot.sendVelocity(0.0, 0.0);
pause(1);

plot(timeArray, distArray, timeArray, srefArray);
