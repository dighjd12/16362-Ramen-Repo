
%% start robot
clc;
robot = neato('exa'); 
mrpl = mrplSystem(); 
startPose = [0.228;0.228;-pi/2];
robot.startLaser();
%% stop robot
robot.close();
robot.shutdown();
clear all;
close all;
clc;
%% initial scan to locate robot
clc;
robot.startLaser();
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
lines_p1 = [[0;0], [0;1.3]];
lines_p2 = [[1.3;0], [0;0]];
robotPose = pose(startPose);
walls = [[1.3; 0.0], [0.0; 0.0], [0.0; 1.3]];
maxIters = 100;
lmLocalizer = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);
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
    %startPose = startPose.poseVec;
    
%% test pickup 1
startPose = [0.228;0.228;-pi/2];
mrpl.setInitialPose(mrpl,startPose);
pickUpLocationList = [[0.3048,1.2198,pi/2];...
                      [0.3048*2,1.2198,pi/2];...
                      [0.3048*3,1.2198,pi/2]];
dropLocationList = [[0.5334,0.3,-pi/2];...
                    [0.5334 + 0.3048*0.5,0.3,-pi/2];...
                    [0.5334 + 0.3048,0.3,-pi/2]];
mrpl.turnRelAngle(mrpl,robot,pi(),0);
mrpl.pickDropObject(mrpl,robot,[0.3048;1.2198;pi/2]);
robot.sendVelocity(0,0);
robot.forksUp();
mrpl.turnRelAngle(mrpl,robot,pi(),0);
mrpl.executeTrajectorySE(mrpl,robot,0.5334,0.3,-pi/2,1); 
robot.forksDown();
%
%% test JS & c task
clc;
close all;
%robot.forksDown();
startPose = [0.228;0.228;-pi/2];
mrpl.setInitialPose(mrpl,startPose);
pickUpLocationList = [[0.29,1.2198,pi/2];...
                      [0.29*2,1.2198,pi/2];...
                      [0.29*3,1.2198,pi/2]];
dropLocationList = [[0.5334,               0.3, -pi/2];...
                    [0.5334 + 0.4048*0.75, 0.33, -pi/2];...
                    [0.5334 + 0.4048*1.5,   0.36, -pi/2]];



Js = jobScheduler(dropLocationList,pickUpLocationList,startPose);
Js.scheduleJob(Js,robot,mrpl);
robot.sendVelocity(0,0);
