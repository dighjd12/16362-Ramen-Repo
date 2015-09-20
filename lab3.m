%% roobot

%********* 1
robot = neato('milli');
%global timeArray;
%global leftArray;
%globa neatoEncoderFrame;

%neatoEncoderFrame=0;
%global neatoEncoderLeft;
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

%global timeArray;
%global leftArray;
%disp(timeArray);
%disp(leftArray);

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
%disp(array);

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

%% challenge task
%set up the listener 
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
%set up vr and vl and t
global timeArray;
global leftArray;

global vlSoFar;
global vrSoFar;
%global xArray;
%global yArray;
%xArray = zeros(1,1);
%yArray = zeros(1,1);


W = 234.95; % robot base width

ks = 0.5;
kv = 0.4;
tf = 12.565*ks/kv;
t = 0; 
vr = double(1000*(0.3*kv + 0.14125*kv/ks * sin(t*kv/(2*ks))));
vl = double(1000*(0.3*kv - 0.14125*kv/ks * sin(t*kv/(2*ks))));
 t0 = 0;
    dt = 0.001;    
time_period = round((tf-t0)/dt);
tic;


%  vlSoFar = zeros(1);
 % vrSoFar = zeros(1);

  x  = zeros(1,time_period);
  y  = zeros(1,time_period);
  
  myPlot = plot(x,y);
  xlim([-0.5 0.5]);
  ylim([-0.5 0.5]);
  
%loop to move the robot by unpdating vl and vr
while (t < tf)
    %drive the robot
    robot.sendVelocity(vl/1000,vr/1000);
    pause(0.001);
    t_i = t;
    t = toc;
    vr = double(1000*(0.3*kv + 0.14125*kv/ks * sin(t*kv/(2*ks))));
    vl = double(1000*(0.3*kv - 0.14125*kv/ks * sin(t*kv/(2*ks))));
  %  plot(timeArray, leftArray);
    
    %get feedback time, left, right
    %calculate vl , vr from feedback
    
    
   
    %get the last value from leftarray/rightarray/timearray
    
    %increment;
    %vlSoFar = [vlSoFar, vlFeedback];
    %vrSoFar = [vrSoFar, vrFeedback];
    
    tfinal = timeArray(length(timeArray));
    
    [x, y, th] = modelDiffSteerRobot(vlSoFar, vrSoFar, t0, tfinal, dt);
    
    set(myPlot, 'Xdata', x.*dt, 'Ydata', y.*dt);
    pause(0.001);
    % put that into modelDiffSteerRobot
    % plot x,y real time
end
robot.sendVelocity(0,0);
%plot(timeArray, leftArray);
robot.encoders.NewMessageFcn = [];



