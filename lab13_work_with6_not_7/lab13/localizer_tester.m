%% start robot
clc;
robot = neato('exa'); 
mrpl = mrplSystem(); 
startPose = [0.228;0.228;pi/2];
robot.startLaser();
%% stop robot
robot.close();
robot.shutdown();
clear all;
close all;
clc;
%% initial scan to locate robot
clc;
startPose = [0.228;0.228;-pi/2];
robot.startLaser();
pause(3);
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
lines_p1 = [[0;0], [0;1.219*2]]; % (0,0) to (0,1.219) - left wall
lines_p2 = [[1.219*2;0], [0;0]];
lines_p3 = [[1.219*2;0], [1.219*2;1.219*2]];
robotPose = pose(startPose);
walls = [[1.219*2; 1.219*2], [1.219*2; 0.0], [0.0; 0.0], [0.0; 1.219*2]];
maxIters = 20;
lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,lines_p3,gain,errThresh,gradThresh);
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
    %mrpl.setInitialPose(mrpl,startPose.poseVec);
    %disp(startPose.poseVec);
    %startPose = startPose.poseVec;
    