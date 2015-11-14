%% tests the findObject function of rangeImage
clc;
close all;

disp('Starting Laser...');
robot.startLaser();
pause(5); % wait for laser data to come in
reading = transpose(double(robot.laser.LatestMessage.Ranges));
image = rangeImage(reading,1,1);
disp('Laser values read');

image.findObject();

%%
robot = neato('hecto');