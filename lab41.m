
%% Lab4  start robot
robot = neato('exa');
robot.sendVelocity(0,0);
%% lab4    
int_dist = 20; %intended dist to travel

% set arrays for ploting
timeArray = zeros(1,1);
errorArray = zeros(1,1); % create 1x1 matrix with value 0
time = 0; %time so far
index = 1; %list index

%PID control constants
kp = 6; %proportional constant
kd = 3; %derivative constant
ki = 0; %integral constant
e_int = 0; %integral of error
e_der = 0; %derivative of error
e_pro = 0; %error 

%encoder readings
leftStart = robot.encoders.LatestMessage.Left; % x(0) position in mm
leftEncoder = leftStart; % x(0) at first in mm
leftGoal = leftStart + int_dist * 10; % distination on x in mm

%set init speed
vr = 0.01; %right wheel speed
vl = 0.01; %left wheel speed

error = int_dist; %diff of distance travel and intended distance
deviation = 0.0001; %error we allow (not the error above)
robot.sendVelocity(vl,vr); %move it
tic; %start clock
while (abs(error) > deviation && time < 6)
    pause(0.01); 
    time = toc; %get time 
    leftEncoder = robot.encoders.LatestMessage.Left;
    error = leftEncoder - leftGoal;
    timeArray(index) = time;
    errorArray(index) = error;
    
    e_pro = error/10; %in cm
    e_int = e_int + error/10; %in cm*s
    e_der = (error-errorArray(end))/10/(time-timeArray(end)); %in cm/s
    u = kp*e_pro + ki * e_int + kd * e_der;
    u = double(u)/100;
    if (abs(u) < 0.3)
        vl = abs(u);
        vr = abs(u);
    else
        vl = 0.3;
        vr = 0.3;
    end
    if error > 0
        robot.sendVelocity(-vl,-vr);
    else
        robot.sendVelocity(vl,vr);
    end
    index = index+1;
    plot(timeArray, errorArray);
end
plot(timeArray, errorArray);
robot.sendVelocity(0,0);
