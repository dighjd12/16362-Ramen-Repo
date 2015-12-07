%% start robot  
robot = neato('pico'); 
%% c
robot.close();
robot.shutdown();
clear all;
close all;

clc;
%% call mrpl system

close all;
mrpl = mrplSystem();

[lp1a,lp1b] = mrpl.executeTrajectory(mrpl,robot,0.7,-0.7+0.145,0.0,1,[0;0;0],[0,0,0]);

%pause(1);

%[lp2a,lp2b] = mrpl.executeTrajectory(mrpl,robot,-0.5,-0.5,-pi()/2.0,1,lp1a,lp1b);

%pause(1);
%disp(lp2);
%mrpl.executeTrajectory(mrpl,robot,-0.25,0.25,pi()/2.0,1,lp2a,lp2b);

%pause(1);
