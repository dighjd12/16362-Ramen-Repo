%% challenge task
robot.startLaser();
pause(5);
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
vGain = 1.0;
%robotPose = pose(15*0.0254, 9*0.0254, pi()/2.0);
robotPose = pose(0.48, 0.33, pi()/2);
lines_p1 = [[0;0], [0;4]];
lines_p2 = [[4;0], [0;0]];
walls = [[4.0; 0.0], [0.0; 0.0], [0.0; 4.0]];
maxIters = 20;

lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

fig = gcf;
driver = robotKeypressDriver(fig);

duration = 60;
time = 0;
tic;
while(time<duration)
    
   % disp('oldPose');
   % disp(robotPose.getPoseVec());
    
    elapsedtime = toc;
    tic;
    time = time + elapsedtime;
    ranges = transpose(double(robot.laser.LatestMessage.Ranges));
    %use every 10th range reading for points
    rangePts = ranges(1:10:length(ranges));
    xPoints = zeros(1,length(rangePts));
    yPoints = zeros(1,length(rangePts));
    i=1;
    for n=1:length(rangePts)
        xPoints(n) = cosd(i)*rangePts(n);
        yPoints(n) = sind(i)*rangePts(n);
       % fprintf('i: %d, point: %d\n', i, rangePts(n));
        i = i+10;
    end
    
    modelPts = [xPoints; yPoints; ones(1,length(xPoints))];
    [success, outPose] = refinePose(lmLocalizer,robotPose,modelPts,maxIters);
    
    robotPose = outPose;
    worldLidarPts = robotModel.senToWorld(robotPose)*modelPts;
  
    %disp('newPose');
   % disp(robotPose.getPoseVec());
    
    %robot
    bodyPts = robotModel.bodyGraph();
    bodyPts1 = bToA(robotPose)*bodyPts;
    robotKeypressDriver.drive(robot,vGain);
    
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
error('end');

%% warm up 2 test

gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
crossPoints = [[0;0;1],[2;0;1],[5;0;1],[-2;0;1],[-5;0;1],[0;0;1],[0;2;1],[0;5;1],[0;-2;1],[0;-5;1]];
lines_p1 = [[0;0], [5;0]];
lines_p2 = [[0;-5], [0;0]]; % at the same ith column, they are the either endpoints of a line segment
pose1 = pose(1,-1,0);

lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

[errPlus0,J] = getJacobian(lmLocalizer,pose1,crossPoints);

p = pose1.getPoseVec();

figure(1)
hold on
plot(crossPoints(1,:),crossPoints(2,:),'-x');
plot(p(1),p(2),'-xb');
plot([p(1),J(1)],[p(2),J(2)],'-r');
hold off

disp(errPlus0);
disp(J);

%% warm up 1-2


gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
% Set up lines
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
% Set up test points
nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);
w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;
modelPts = [x1pts ; y1pts ; w1pts];
% pick a pose
dx = -0.05*rand();
dy = -0.05*rand();
dt = -0.05+0.2*rand();
thePose = pose(0.0+dx,0.0+dy,0.0+dt);

lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

[errPlus0,J] = getJacobian(lmLocalizer,thePose,modelPts);

p = thePose.getPoseVec();

figure(1)
hold on
plot(modelPts(1,:),modelPts(2,:),'-x');
for i=1:length(lines_p1)
    plot([lines_p1(1,i) lines_p2(1,i)],[lines_p1(2,i) lines_p2(2,i)], '-r');
end
plot(p(1),p(2),'-xb');
plot([p(1),J(1)],[p(2),J(2)],'-m');
hold off

disp(errPlus0);
disp(J);


%% warm up 3 test
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
% Set up lines
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
% Set up test points
nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);
w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;
modelPts = [x1pts ; y1pts ; w1pts]; %size 30
% pick a pose
dx = -0.05*rand();
dy = -0.05*rand();
dt = -0.05+0.2*rand();
thePose = pose(0.01+dx,0.02+dy,0.0+dt);
p1 = thePose.getPoseVec();
maxIters = 50; 

fprintf('inPose: %d, %d, %d\n', p1(1,1),p1(2,1),p1(3,1));

lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
[success, outPose] = refinePose(lmLocalizer,thePose,modelPts,maxIters);

p = outPose.getPoseVec();


fprintf('success: %d\n', success);
fprintf('outPose: %d, %d, %d\n', p(1,1),p(2,1),p(3,1));

%%

robot = neato('exa');
robot.startLaser();
pause(5);





