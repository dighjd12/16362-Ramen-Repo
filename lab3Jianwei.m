%% start the robot
robot = neato('milli');
%% exercise 1 robot listener
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
tic;
t = toc;
disp(class(robot));
while (t < 5)
    t = toc;
    pause(0.001);
end
disp('end');
robot.encoders.NewMessageFcn = [];
robot.sendVelocity(0,0);
%% exercise 2 v vs t case 1
t_limit = 15;
vl = 0.01;
vr = 0.01;
dv = zeros(1,1);
dt = zeros(1,1);
left_i = robot.encoders.LatestMessage.Left;
left_temp = 0;
i = 1;
while (t_i < t_limit)
    robot.sendVelocity(vl,vr);
    pause(1);
    left_f = robot.encoders.LatestMessage.Left;
    t_f = toc;
    dt(i) = t_f;
    dv(i) = (left_f-left_i)/(t_f-t_i);
    disp((left_f-left_i)/(t_f-t_i));
    left_i = left_f;
    t_f = t_i;
    i = i+1;
    disp(class(robot.encoders));
    plot(dt,dv);
end
plot(dt,dv);
disp('end');
robot.sendVelocity(0,0);
%% exercise 2 v vs t case 1
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
%global variables
tic;
t = toc;

disp(class(robot));
while (t < 5)
    t = toc;
    pause(0.001);
end
disp('end');
robot.encoders.NewMessageFcn = [];
robot.sendVelocity(0,0);

%% challenge task
%set up the listener 
robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
%set up vr and vl and t
global timeArray;
global leftArray;
global xArray;
global yArray;
xArray = zeros(1,1);
yArray = zeros(1,1);
ks = 0.5;
kv = 0.4;
tf = 12.565*ks/kv;
t = 0; 
vr = double(1000*(0.3*kv + 0.14125*kv/ks * sin(t*kv/(2*ks))));
vl = double(1000*(0.3*kv - 0.14125*kv/ks * sin(t*kv/(2*ks))));
tic;
%loop to move the robot by unpdating vl and vr
while (t < tf)
    robot.sendVelocity(vl/1000,vr/1000);
    pause(0.001);
    t_i = t;
    t = toc;
    vr = double(1000*(0.3*kv + 0.14125*kv/ks * sin(t*kv/(2*ks))));
    vl = double(1000*(0.3*kv - 0.14125*kv/ks * sin(t*kv/(2*ks))));
    plot(timeArray, leftArray);
    
    %get feedback time, left, right
    %calculate vl , vr
    % put that into modelDiffSteerRobot
    % plot x,y real time
end
robot.sendVelocity(0,0);
plot(timeArray, leftArray);
robot.encoders.NewMessageFcn = [];


