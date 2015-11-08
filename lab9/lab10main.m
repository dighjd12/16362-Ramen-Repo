%% challenge task

mrpl = mrplSystem();
robot = neato('zetta');
robot.startLaser();
pause(5);

%% challenge task

robotPose = pose(0.5,0.5,pi()/2.0);

[lpb1] = mrpl.executeTrajectorySE(mrpl,robot,0.25,0.75,pi()/2.0,1,robotPose.getPoseVec());

pause(1);

%%

[lpb2,lpc2] = executeTrajectorySE(mrpl,robot,0.75,0.25,0.0,1,lpb1,lpd);
pause(1);

executeTrajectorySE(mrpl,robot,0.5,0.5,pi()/2.0,1,lpb2,lpd);
pause(1);


%%

mrpl = mrplSystem();

[lp1a,lp1b] = mrpl.executeTrajectory(mrpl,robot,0.25,0.25,0.0,1,[0;0;0],[0,0,0]);

gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
vGain = 1.0;

lines_p1 = [[0;0], [0;4]];
lines_p2 = [[4;0], [0;0]];
walls = [[4.0; 0.0], [0.0; 0.0], [0.0; 4.0]]; %%
maxIters = 20; %%
initMaxIters = 50;

poseInit = pose(15*0.0254, 9*0.0254, pi()/2.0); % initial pose, given by the odometry
k = 0.25;
poseFused = poseInit;
poseLidar = [0;0;0];

lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

fig = gcf;

xRealArray = poseInit(1,1);
yRealArray = poseInit(2,1);
thRealArray = poseInit(3,1);

% read the value from the encoder
%encoder reads distance in mm, to get in m, divide by 1000
leftLast = double(double(robot.encoders.LatestMessage.Left)/1000);
leftNow = double(leftLast); 
rightLast = double(double(robot.encoders.LatestMessage.Right)/1000);
rightNow = double(rightLast);

% distance the left and right wheels traveled in dt
lds = 0;
rds = 0;

i=2;

duration = 60;
time = 0;
tic;
while(time<duration)
    
    %%%%%%%%%%%%%%%%%%%%%processOdometryData
    %update odometry pose
    leftNow = double(double(robot.encoders.LatestMessage.Left)/1000); 
    lds = double(leftNow - leftLast);
    leftLast = double(leftNow);

    rightNow = double(double(robot.encoders.LatestMessage.Right)/1000);
    rds = double(rightNow - rightLast); 
    rightLast = double(rightNow);
    
    real_vl = double(double(lds)/dtime); 
    real_vr = double(double(rds)/dtime);
    [V,w] = robotModel.vlvrToVw(real_vl,real_vr);
    dth = w*dtime;
    %last angle the robot is pointing
    th = obj.thRealArray(i-1);
    
    obj.thRealArray(i) = obj.thRealArray(i-1)+dth;
    obj.xRealArray(i) = obj.xRealArray(i-1) + double(V*cos(th)*dtime);
    obj.yRealArray(i) = obj.yRealArray(i-1) + double(V*sin(th)*dtime);
    
    robot_th = thRealArray(i);
    robot_x = xRealArray(i);
    robot_y = yRealArray(i);
    robot_pose = [robot_x ;robot_y ;robot_th]; %from encoder
    
    poseFused = robot_pose;
    
    dtime = toc;
    tic;
    time = time + dtime;
    
    
    
    %%%%%%%%%%%%%%%%%%processRangeImage
    ranges = transpose(double(robot.laser.LatestMessage.Ranges));
    %use every 10th range reading for points
    rangePts = ranges(1:10:length(ranges));
    xPoints = zeros(1,length(rangePts));
    yPoints = zeros(1,length(rangePts));
    j=1;
    for n=1:length(rangePts)
        xPoints(n) = cosd(j)*rangePts(n);
        yPoints(n) = sind(j)*rangePts(n);
       % fprintf('i: %d, point: %d\n', i, rangePts(n));
        j = j+10;
    end
    
    modelPts = [xPoints; yPoints; ones(1,length(xPoints))];
    
    
    %%%%%%%%%%%%%%%%%%%%pose fusion based on processed odometry data and
    %%%%%%%%%%%%%%%%%%%%range image
    [success, outPose] = refinePose(lmLocalizer,poseFused,modelPts,maxIters);
    
    %fixing poseFused
    poseLidar = outPose;
    poseFused(1,1) = poseFused(1,1) + k*(poseLidar(1,1)-poseFused(1,1)); %change x
    poseFused(2,1) = poseFused(2,1) + k*(poseLidar(2,1)-poseFused(2,1)); %change y
    th2 = poseLidar(3,1);
    th1 = poseFused(3,1);
    poseFused(3,1) = poseFused(3,1) + k*(atan2(sin(th2-th1),cos(th2-th1)));
    
    %setpose
    %??????
    
    %%%%%%%%%%%%%%%%%%%%%% part of processOdometryData
    obj.thRealArray(i) = poseFused(1,1);
    obj.xRealArray(i) = poseFused(2,1);
    obj.yRealArray(i) = poseFused(3,1);
    i = i+1;
    
    %disp(poseInit.getPoseVec());
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%plotting below 
    worldLidarPts = robotModel.senToWorld(poseFused)*modelPts;
    bodyPts1 = bToA(poseFused)*robotModel.bodyGraph();
    
    %%%%%%%%%plotting%%%%%%%%%%
    figure(1)
    plot(walls(1,:), walls(2,:), '-b'); %plot walls
    hold on
    plot(bodyPts1(1,:),bodyPts1(2,:),'-g'); %robotPoints
    plot(worldLidarPts(1,:), worldLidarPts(2,:), '-xr'); %plot lidar points in sensor frame
    hold off
    %%%%%%%%%plotting%%%%%%%%%%
    pause(0.001);
    
end

robot.sendVelocity(0,0);