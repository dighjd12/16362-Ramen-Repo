%% start robot  
robot = neato('sim'); 
%% c
robot.close();
robot.shutdown();
clear all;
close all;

clc;
%% call mrpl system

close all;
mrpl = mrplSystem();

%real and ref
mrpl.executeTrajectory(mrpl,robot,0.25,0.25,0.0,1);

pause(1);

mrpl.executeTrajectory(mrpl,robot,-0.5,-0.5,-pi()/2.0,1);

pause(1);
%disp(lp2);
mrpl.executeTrajectory(mrpl,robot,-0.25,0.25,pi()/2.0,1);

pause(1);
