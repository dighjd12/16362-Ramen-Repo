%% testCubicSpiral

clc;
close all;

profile on;
curve = cubicSpiral([-3.0 1.55 3.0],10001);
curve.makeLookupTable(10);

%profile viewer;

%plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));
%% testCubicSpiral


curve = cubicSpiral([3.0 1.55 3.0],10001);
%curve = cubicSpiral([-1 1 1],10001);
pose = curve.getFinalPose();
fprintf('x:%f y:%f t:%f\n', pose(1),pose(2),pose(3));

disp(curve.distArray(100));
disp(curve.poseArray(3,100));

plot(curve.poseArray(1,1:end),curve.poseArray(2,1:end));