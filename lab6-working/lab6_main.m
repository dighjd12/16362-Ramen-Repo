%% start robot
robot = neato('sim');
%% c
robot.close();
robot.shutdown();
clear all;
close all;
%% call mrpl system

close all;

n=2;

mrplSystem.executeTrajectory(robot,0.25,0.25,0.0,1,n);

%%

pause(3);

n = 3;

mrplSystem.executeTrajectory(robot,-0.5,-0.5,-pi()/2.0,1,n);

pause(3);

n = 4;

mrplSystem.executeTrajectory(robot,-0.25,0.25,pi()/2.0,1,n);

pause(3);
