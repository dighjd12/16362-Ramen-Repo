%% modelDiffSteerRobot
% Function:     modelDiffSteerRobot
% Description:  takes wheel velocities as an input and 
%               calculates the robot trajectory 
% Takes:  
%	vl - vector of left wheel velocity in mm/sec
%	vr - vector of right wheel velocity in mm/sec
%   t0 - initial time
%   tf - final time
%   dt - integration time step
% Returns: 
%   x  - vector of x position over time
%   y  - vector of y position over time
%   th - vector of angle over time
% Last Edit:    9/19/2015
function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
  % set up return values - set all to 0 to start
  time_period = round((tf-t0)/dt);
  W = 234.95; % robot base width

  if time_period <= 0
    % do something
  end

  % create empty vectors to optimize odometry loop
  x  = zeros(1,length(vr));
  y  = zeros(1,length(vr));
  th = zeros(1,length(vr));
  
  %% Initialize Robot Starting Position and Angle 
  % initialize all to 0
  x(1) = 0;  
  y(1) = 0; 
  th(1) = 0;
  
  %% Update Robot Position
  % update linear and angular velocity, then update position starting
  % with the new angle
  
 % myPlot = plot(x,y);
 % xlim([-0.5 0.5]);
 % ylim([-0.5 0.5]);
  
  for k = 1:(length(vr)-1)  %time_period+1
    % update linear and angular velocity of robot
    V = (vr(k) + vl(k))/2;
    omega = (vr(k) - vl(k))/W;  
    
    % calculate next position    
    th(k+1) = th(k) + omega       *dt;    
    x(k+1)  = x(k)  + V*cos(th(k))*dt;    
    y(k+1)  = y(k)  + V*sin(th(k))*dt;
    
   % set(myPlot, 'Xdata', x.*dt, 'Ydata', y.*dt);
   % pause(0.001);
  end
  
end