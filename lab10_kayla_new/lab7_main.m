 %% start robot   
 robot = neato('pico'); 
 %% c
 robot.close();
% robot.shutdown();
% clear all;
% close all;
% 
%%



%% initial scan to locate robot
clc;
robot.stopLaser();
gain = 0.01;
errThresh = 0.001;
gradThresh = 0.0005;
lines_p1 = [[0;0], [0;4]];
lines_p2 = [[4;0], [0;0]];
robotPose = pose(0.5, 0.5, pi/2);
walls = [[4.0; 0.0], [0.0; 0.0], [0.0; 4.0]];
maxIters = 40;
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

%% drive robot in world frame
close all;
clc;
mrpl = mrplSystem();
startPose = [0.5;0.5;pi/2]; % the pose we start in the world frame
pose1 = [0.25;0.75;pi/2]; % first destination in world frame
pose2 = [0.75;0.25;0]; % second destination in world frame
pose3 = [0.5;0.5;pi/2]; % final destination in world frame
%startPose = [0.5;0.5;pi/2];
% startPose = startPose.poseVec;
% mat_f_to_w = [cos(startPose(3)), -sin(startPose(3)), startPose(1);
%               sin(startPose(3)),  cos(startPose(3)), startPose(2);
%               0                ,  0                , 1          ];
% mat_w_to_f = mat_f_to_w^-1;
% r1 = (mat_w_to_f)*[pose1(1);pose1(2);1];
% r2 = (mat_w_to_f)*[pose2(1);pose2(2);1];
% r3 = (mat_w_to_f)*[pose3(1);pose3(2);1];
% th1 = atan2(sin(-startPose(3) + pose1(3)),cos(-startPose(3)+pose1(3)));
% th2 = atan2(sin(-startPose(3) + pose2(3)),cos(-startPose(3)+pose2(3)));
% th3 = atan2(sin(-startPose(3) + pose3(3)),cos(-startPose(3)+pose3(3)));
% pose1 = [r1(1);r1(2);th1];
% pose2 = [r2(1);r2(2);th2];
% pose3 = [r3(1);r3(2);th3];

mrpl.setInitialPose(mrpl,startPose);

mrpl.executeTrajectorySE(mrpl,robot,pose1(1),pose1(2),pose1(3),1);

pause(2);

mrpl.executeTrajectorySE(mrpl,robot,pose2(1),pose2(2),pose2(3),1)%-pi()/2.0,1);
pause(2);
%disp('3');
mrpl.executeTrajectorySE(mrpl,robot,pose3(1),pose3(2),pose3(3),1);

%pause(1);
