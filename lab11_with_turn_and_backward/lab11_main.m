%% start robot
clc;
robot = neato('yotta'); 
robot.startLaser();
%% stop robot
robot.close();
robot.shutdown();
clear all;
close all;
clc;

%% changle task 
clc;
close all;
mrpl = mrplSystem();
disp('Starting Laser...');
pause(5); % wait for laser data to come in
reading = transpose(double(robot.laser.LatestMessage.Ranges));
image = rangeImage(reading,1,1); 
disp('Laser values read');
disp('Finding Best Midpoint...');
[middle,bpose] = image.findObject();
fprintf('board pose found: %d %d %d', bpose(1),bpose(2),bpose(3));
bInSen = pose(bpose);
rInSen = pose([0.11;0;0]);
bInG = pose([0.038+0.12;0;0]);
goalPoseInr = pose.matToPoseVec(rInSen.aToB() * bInSen.bToA() * bInG.aToB());
goalInr = pose(goalPoseInr);
startPose = [0.5;0.5;pi/2];
mrpl.setInitialPose(mrpl,startPose);
rInw = pose(mrpl.follower.lastPoser);
gInw = pose.matToPoseVec(rInw.bToA()*goalInr.bToA());
mrpl.executeTrajectorySE(mrpl,robot,gInw(1),gInw(2),gInw(3),1);

%% 
mrpl.moveRelDistance(mrpl,robot,0.15,0);

%%
mrpl.turnRelAngle(mrpl,robot,pi(),0);
