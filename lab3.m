%% roobot

%********* 1
robot = neato('deca');
%global timeArray;
%global leftArray;
%globa neatoEncoderFrame;
%neatoEncoderFrame=0;
global neatoEncoderLeft;
%global neatoEncoderRight;

%% encoder function

%********* 2
%currentFrame = neatoEncoderFrame;
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;

%% 

robot.encoders.NewMessageFcn=[];

%% move the robot and get array valuees

%********* 3
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
robot.sendVelocity(0.05,0.05);

loop = 0;
time=0;
while time < 5 % for two seconds
    elapsedTic = tic;
    pause(0.001);
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    loop = loop+1;
end
robot.encoders.NewMessageFcn= [];

global timeArray;
global leftArray;
disp(timeArray);
disp(leftArray);

%% case2 for warmup 2?

%********* 4
[timeArrayTrimmed, ia, ic] = unique(timeArray); % returns unique elements of time array
leftArrayTrimmed = leftArray(ia); % returns corresponding left values

initialTime = timeArrayTrimmed(1);

accumulatedTime = 1:(length(timeArrayTrimmed));
accumulatedTime(1) = 0;
v = 1:(length(timeArrayTrimmed));

for n = 2: (length(timeArrayTrimmed))
    dt = timeArrayTrimmed(n) - timeArrayTrimmed(n-1);
    accumulatedTime(n) = accumulatedTime(n-1)+dt;
    ds = leftArrayTrimmed(n) - leftArrayTrimmed(n-1);
    v(n) = ds./dt;
end

accumulatedTimeFinal = accumulatedTime(2:length(timeArrayTrimmed)); %first value is duplicate
%timeArrayFinal = timeArrayTrimmed(2:length(timeArrayTrimmed));
v = v(2:length(timeArrayTrimmed)); % make same length
plot(accumulatedTimeFinal, v);

%% stop

robot.encoders.NewMessageFcn= [];
disp(array);

%% warm up 1

%lh = event.listener (robot.encoders, 'OnMessageReceived', @neatoEncoderEventListener);


%robot.laser.NewMessageFcn=@neatoLaserEventListener

pause(3);

%enc = rossubscriber('/enc', @neatoEncoderEventListener);
robot.sendVelocity(0.05,0.05);


pause(3);

robot.sendVelocity(0.0,0.0);

%disp(neatoEncoderDataTimestamp);

%robot.sendVelocity(0.01,0.01);

%disp(neatoEncoderDataTimestamp);


