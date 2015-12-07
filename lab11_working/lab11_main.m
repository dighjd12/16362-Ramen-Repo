%% start robot
clc;
robot = neato('centi'); 
mrpl = mrplSystem(); 
startPose = [0.5;0.5;pi/2];
robot.startLaser();
%% stop robot
robot.close();
robot.shutdown();
clear all;
close all;
clc;
%% initial scan to locate robot
clc;
robot.stopLaser();
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
lines_p1 = [[0;0], [0;1.3]];
lines_p2 = [[1.3;0], [0;0]];
robotPose = pose(0.5, 0.5, pi/2);
walls = [[1.3; 0.0], [0.0; 0.0], [0.0; 1.3]];
maxIters = 100;
lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
robot.startLaser();
pause(5);
ranges = transpose(double(robot.laser.LatestMessage.Ranges));
ranges(ranges > 3) = 0;
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
startPose = outPose;

    bodyPts = robotModel.bodyGraph();
    bodyPts1 = bToA(robotPose)*bodyPts;
        worldLidarPts = robotModel.senToWorld(robotPose)*modelPts;

    
    %%%%%%%%%plotting%%%%%%%%%%
    figure(1)
    plot(walls(1,:), walls(2,:), '-b'); %plot walls
    hold on
    plot(bodyPts1(1,:),bodyPts1(2,:),'-g'); %robotPoints 
    plot(worldLidarPts(1,:), worldLidarPts(2,:), '-xr'); %plot lidar points in sensor frame
    hold off
    %robot.stopLaser();
    mrpl.setInitialPose(mrpl,startPose.poseVec);
    disp(startPose.poseVec);




%%
startPose = [0.5;0.5;pi/2];
mrpl.setInitialPose(mrpl,startPose);
    
    %% changle task 
clc;
close all;
mrpl = mrplSystem();
disp('Starting Laser...');
robot.startLaser();
pause(5); % wait for laser data to come in
rInw = pose(mrpl.follower.lastPoser);   
rInSen = pose([0.11;0;0]);
reading = transpose(double(robot.laser.LatestMessage.Ranges));
image = rangeImage(reading,1,1); 
disp('Laser values read');
disp('Finding Best Midpoint...');
[middle,bpose] = image.findObject(rInSen,rInw);
fprintf('board pose found: %d %d %d', bpose(1),bpose(2),bpose(3));
bInSen = pose(bpose);
bInG = pose([0.038+0.15;0;0]);
goalPoseInr = pose.matToPoseVec(rInSen.aToB() * bInSen.bToA() * bInG.aToB());
goalInr = pose(goalPoseInr);
%startPose = [0.5;0.5;pi/2];
%mrpl.setInitialPose(mrpl,startPose);                                                                                                                
gInw = pose.matToPoseVec(rInw.bToA()*goalInr.bToA());
mrpl.executeTrajectorySE(mrpl,robot,gInw(1),gInw(2),gInw(3),1); 
robot.stopLaser();


%mrpl.moveRelDistance(mrpl,robot,0.15,0);

%mrpl.turnRelAngle(mrpl,robot,pi(),0);
