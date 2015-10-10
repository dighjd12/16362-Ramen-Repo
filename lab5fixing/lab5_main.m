%% test integrating figure 8 trajectory
fig_ref = figure8ReferenceControl(0.4,0.8,0.5);
dt = 1e-3;
t = 0;
tf = fig_ref.getTrajectoryDuration(fig_ref);
xArray = zeros(1,1);
yArray = zeros(1,1);
thArray = zeros(1,1);
index = 2;
while t < tf
    t = t + dt;
    [V,w] = fig_ref.computeControl(fig_ref,t);
    disp_th = double(w * dt);
    thArray(index) = thArray(end) + disp_th;
    th = thArray(index-1);
    dist = V * dt;
    disp_y = dist * sin(th);
    disp_x = dist * cos(th);
    xArray(index) = xArray(end) + disp_x;
    yArray(index) = yArray(end) + disp_y;
    index = index + 1;
end
plot(xArray,yArray);

%% testing the robotTraj generate correct 8 shape
ref_ctrl = figure8ReferenceControl(0.4,0.4,0.4);
duration = ref_ctrl.getTrajectoryDuration(ref_ctrl);
Traj = robotTrajectory(0, duration, 1e-3, 0, [0 0 0], ref_ctrl);
plot(Traj.x, Traj.y);

%% robot stuff
    

robot = neato('sim');

%% robot stuff 2
    
robot.close()
robot.shutdown() 
clear all;
            
%% call trajectory follower here
close all;
ctrl = controller(0.004,0.0025);

fig_ref = figure8ReferenceControl(0.4,0.4,0.5); 
%fig_ref = trapezoidalStepReferenceControl(.5,.75,.25,1, 1);

follower = trajectoryFollower(ctrl, fig_ref);
follower.feedForward(robot, follower,true,0.05);
 


