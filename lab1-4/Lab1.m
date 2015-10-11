%% Lab1 Task1 Move the Robot

robot = neato('nano');

%Case 1
tstart = tic;
while toc(tstart)< 4
    robot.sendVelocity(0.05, 0.05);
    pause(0.005);
end
robot.sendVelocity(0, 0);

%Case 2
tstart = tic;
while toc(tstart)< 4
    robot.sendVelocity(-0.05, -0.05);
    pause(0.005);
end
robot.sendVelocity(0, 0);

clearall;

%% Lab1 Task2

robot = neato('sim');

leftStart = 20;%arbitrary number
leftEncoder = leftStart; % in "mm"

signedDistance =0;
time = 0;

while signedDistance < 10
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    robot.sendVelocity(0.050, 0.050);
    leftEncoder = leftEncoder + elapsedTime.*50; %adds travelled distance in mm
    signedDistance = (leftEncoder - leftStart)/10; %in cm
    disp(signedDistance);
    
end

robot.sendVelocity(0.0, 0.0);


%% Lab1 Task3

robot = neato('sim');
timeArray = zeros(1,1);
distArray = zeros(1,1); % create 1x1 matrix with value 0

leftStart = 20;%arbitrary number
leftEncoder = leftStart; % in "mm"


%startTime = tic;
%elapsedTic = tic;
signedDistance =0;
arrayIndex = 1; %index starts with 1

time = 0;

while signedDistance < 10
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    robot.sendVelocity(0.050, 0.050);
    leftEncoder = leftEncoder + elapsedTime.*50; %adds travelled distance in mm
    signedDistance = (leftEncoder - leftStart)/10; %in cm
    disp(signedDistance);
    
  %  timeArray(arrayIndex) = elapsedTime; % in sec
   % distArray(arrayIndex) = elapsedTime.*50; %in cm
    
    timeArray(arrayIndex) = time; %total elapsed time so far
    distArray(arrayIndex) = signedDistance; %total travelled distance so far
    arrayIndex = arrayIndex + 1;
    
    
end

robot.sendVelocity(0.0, 0.0);
plot(timeArray, distArray);

%clear all;


%% Lab1 Challenge

robot = neato('sim');
int_dist = 20;
timeArray = zeros(1,1);
distArray = zeros(1,1); % create 1x1 matrix with value 0

leftStart = robot.encoders.LatestMessage.Left;
leftEncoder = leftStart; % in "mm"

signedDistance =0;
arrayIndex = 1; %index starts with 1

time = 0;

while signedDistance < int_dist
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    robot.sendVelocity(0.050, 0.050);
    leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
    signedDistance = (leftEncoder - leftStart)/10; %in cm
    disp(robot.encoders.LatestMessage.Left);
    
    timeArray(arrayIndex) = time; %total elapsed time so far
    distArray(arrayIndex) = signedDistance; %total travelled distance so far
    arrayIndex = arrayIndex + 1;
end

robot.sendVelocity(0.0, 0.0);
pause(2);

leftStart = leftEncoder;
signedDistance = 0;

while abs(signedDistance) < int_dist
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    robot.sendVelocity(-0.050, -0.050);
    leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
    signedDistance = (leftEncoder - leftStart)/10; %in cm
    disp(robot.encoders.LatestMessage.Left);
    
    timeArray(arrayIndex) = time; %total elapsed time so far
    distArray(arrayIndex) = int_dist-abs(signedDistance); %total travelled distance so far
    arrayIndex = arrayIndex + 1;
    
end
robot.sendVelocity(0.0, 0.0);
disp(robot.encoders.LatestMessage.Left);

%plot(timeArray, distArray,'re.');


