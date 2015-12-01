%% robot

%%

tPause = 2;
vmax = .25;
amax = .75;
dist = 1;
angle = pi();
wmax = pi()/2;
awmax = 3*pi()/2;
sign = -1;

tStepCtrl = trapezoidalStepReferenceControl(tPause, amax, vmax, dist, sign);
tTurnCtrl = trapezoidalTurnReferenceControl(0, awmax, wmax, angle, sign);

%%

tPause = 2;
vmax = .25;
amax = .75;
dist = 1;
sign = -1;
tStepCtrl = trapezoidalStepReferenceControl(tPause, amax, vmax, dist, sign);

time=0;
duration = tStepCtrl.getTrajectoryDuration(tStepCtrl);
tdelay = 0.003;
e_int = 0; %integral of error
e_der = 0; %derivative of error
e_pro = 0; %error 
kp = 6; %proportional constant
kd = 0.0125; %derivative constant
ki = 0.00001;
errorArray = zeros(1,1);
feedback = true;

leftStart = double(robot.encoders.LatestMessage.Left);
leftEncoder = double(leftStart);

%test tStepCtrl
elapsedTic = tic;
while time < duration
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    elapsedTic = tic;
    
    [V,w] = tStepCtrl.computeControl(tStepCtrl,time);
    [vl,vr] = robotModel.VwTovlvr(robotModel,V,w);
    
    sdelay = trapezoidalDistanceProfile((time-tdelay), amax, vmax, dist, sign);
    
    upid = 0;
    leftEncoder = robot.encoders.LatestMessage.Left; %adds travelled distance in mm
    signedDistance = double(double(leftEncoder - leftStart)/1000); %in m
    
    if(feedback)
        error = double(sdelay - signedDistance);
        e_pro = double(error/1000); %in m
        e_int = double(e_int) + error/1000; %in m*s
        e_der = double((error-errorArray(end))/1000/(time-timeArray(end))); %in m/s
        upid = kp*e_pro + ki * e_int + kd * e_der;
    end
    ureal = vl + upid;
    
    robot.sendVelocity(ureal, ureal);
    
end
robot.sendVelocity(0,0);

%%

%test tTurnCtrl

tPause = 2;
angle = pi();
wmax = pi()/2;
awmax = 3*pi()/2;
sign = -1;
delay = 0.003;
tTurnCtrl = trapezoidalTurnReferenceControl(0, awmax, wmax, angle, sign);
feedback = true;
time=0;
duration = tTurnCtrl.getTrajectoryDuration(tTurnCtrl);

kp = 0.05; %proportional constant
kd = 0.05; %derivative constant
ki = 0.00001;

thErrorArray = zeros(1,1);

leftLast  = double(double(robot.encoders.LatestMessage.Left)/1000);
leftNow   = double(leftLast); 
rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
rightNow  = double(rightLast);

% distance the left and right wheels traveled in dt
lds = 0;
rds = 0;

timeArray = zeros(1,1);
wArray = zeros(1,1);
thArray = zeros(1,1);
threfArray = zeros(1,1);
i=2;

elapsedTic = tic;
while time < duration
    elapsedTime = toc(elapsedTic); %elapsedTime between this loop and the previous one
    time = time + elapsedTime;
    elapsedTic = tic;
    
    dtime = time - timeArray(end);

    [V,w] = tTurnCtrl.computeControl(tTurnCtrl,time+delay);
    threfArray(i) = threfArray(i-1) + w*dtime;
    
    w_control = 0;

    % read encoder and update old reading
    %(of course, still in mm, divide by 1000)
    leftNow  = double(double(robot.encoders.LatestMessage.Left)/1000); 
    lds      = double(leftNow - leftLast);
    leftLast = double(leftNow);

    rightNow  = double(double(robot.encoders.LatestMessage.Right)/1000);
    rds       = double(rightNow - rightLast); 
    rightLast = double(rightNow);

    %get the real vl and vr
    real_vl = double(double(lds)/dtime); 
    real_vr = double(double(rds)/dtime);
    [V_real,w_real] = robotModel.vlvrToVw(robotModel,real_vl,real_vr);
    dth = w_real*dtime;
    thArray(i) = thArray(i-1) + dth;
    
    if(feedback)
        %atan2?
        thErrorArray(i) = threfArray(i) - thArray(i);
        th_i = sum(thErrorArray);
        th_d = (double(thErrorArray(end))-double(thErrorArray(end-1)))/dtime;
        th_p = thErrorArray(end);
        th_c = kp * th_p + ki * th_i + kd * th_d;
           
        w_control = th_c;
    end
   
    
    [vl,vr] = robotModel.VwTovlvr(robotModel,V,w);
    if(abs(vl)>0.3)
        vl = 0.3;
    end
    if(abs(vr)>0.3)
        vr = 0.3;
    end
    
    timeArray(i) = time;
    wArray(i) = w;
    i = i+1;
    
    robot.sendVelocity(vl,vr);
end
robot.sendVelocity(0,0);