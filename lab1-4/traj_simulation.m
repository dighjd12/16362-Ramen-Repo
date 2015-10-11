% Function:     traj_simulation
% Description:  demonstrates function of modelDiffSteerRobot.m 
% Takes:        
%   none
% Returns:      
%   none
% Last Edit:    9/19/2015

function traj_simulation()    
    % clear everything
    clc;
    close all;
    clear all;
    
    % change colormap
    CT = cbrewer('qual', 'Dark2', 4);
    colormap(CT);

    % constant across both clothoid and figure eight
    t0 = 0;
    dt = 0.001;
 
    
    %% clothoid
    
    t0 = 0;
    dt = 0.001;
    
    k = 2;
    tf = 11.207*k;
    t = t0:dt:tf;
    v_r = 1000 .* (0.1/k + 0.01174.*t./(k^2));
    v_l = 1000 .* (0.1/k - 0.01174.*t./(k^2));

    [x, y, th] = modelDiffSteerRobot(v_l, v_r, t0, tf, dt);

    x_clothoid = x.*dt;
    y_clothoid = y.*dt;
    
    %% figure eight
    
    t0 = 0;
    dt = 0.001;
    
    k_s = 0.5;
    k_v = 0.4;
    tf = round(12.565 * k_s/k_v);
    t = t0:dt:tf;
    
    v_r = 1000 .* (0.3*k_v + 0.14125*(k_v/k_s) * sin((t.*k_v)./(2*k_s)));
    v_l = 1000 .* (0.3*k_v - 0.14125*(k_v/k_s) * sin((t.*k_v)./(2*k_s)));
    
    [x, y, th] = modelDiffSteerRobot(v_l, v_r, t0, tf, dt);
  %  x_figure8 = x.*dt;
   % y_figure8 = y.*dt;

    %% plot clothoid
    figure(1);
    hold on;
    hPos = line(x_clothoid, y_clothoid);
    
    hTitle  = title ('16-362 Lab 3 Simulation: Clothoid');
    hXLabel = xlabel('x position ($m$)'                 );
    hYLabel = ylabel('y position ($m$)'                 );
    
    % formatting
    set(hPos, ...
        'Color'      , CT(1,:)       ,...
        'LineWidth'  , 2             );
    set(gca                          ,...
        'Box'        , 'off'         ,...
        'TickDir'    , 'in'          ,...
        'TickLength' , [0.02 0.02]   ,...
        'XMinorTick' , 'on'          ,...
        'YMinorTick' , 'on'          ,...
        'XColor'     , [0.3 0.3 0.3] ,...
        'YColor'     , [0.3 0.3 0.3] ,...
        'YTick'      , 0:0.1:.5      ,...
        'XTick'      , 0:0.1:.5      ,...
        'LineWidth'  , 1 );
    set([hTitle, hXLabel, hYLabel]   ,...
        'Interpreter','LaTex'        );
    set([hXLabel, hYLabel]           ,...
        'FontSize'   , 12            );
    set(hTitle                       ,...
        'FontSize'   , 14            ,...
        'FontWeight' , 'bold'        );
    hold off;
    
    %% plot figure eight
    figure(2);
    hold on;
    hPos = line(x_figure8, y_figure8);
    
    hTitle  = title ('16-362 Lab 3 Simulation: Figure Eight');
    hXLabel = xlabel('x position ($m$)'                     );
    hYLabel = ylabel('y position ($m$)'                     );
    
    % formatting
    set(hPos, ...
        'Color'      , CT(2,:)       ,...
        'LineWidth'  , 2             );
    set(gca                          ,...
        'Box'        , 'off'         ,...
        'TickDir'    , 'in'          ,...
        'TickLength' , [0.02 0.02]   ,...
        'XMinorTick' , 'on'          ,...
        'YMinorTick' , 'on'          ,...
        'XColor'     , [0.3 0.3 0.3] ,...
        'YColor'     , [0.3 0.3 0.3] ,...
        'YTick'      , -.5:0.1:.5    ,...
        'XTick'      , -.5:0.1:.5    ,...
        'LineWidth'  , 1 );
    set([hTitle, hXLabel, hYLabel]   ,...
        'Interpreter','LaTex'        );
    set([hXLabel, hYLabel]           ,...
        'FontSize'   , 12            );
    set(hTitle                       ,...
        'FontSize'   , 14            ,...
        'FontWeight' , 'bold'        );
    hold off;
end

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
  time_period = (tf-t0)/dt;
  W = 234.95; % robot base width

  if time_period <= 0
    % do something
  end

  % create empty vectors to optimize odometry loop
  x  = zeros(1,time_period);
  y  = zeros(1,time_period);
  th = zeros(1,time_period);

  %% Initialize Robot Starting Position and Angle 
  % initialize all to 0
  x(1) = 0;  
  y(1) = 0; 
  th(1) = 0;
  
  %% Update Robot Position
  % update linear and angular velocity, then update position starting
  % with the new angle
  for k = 1:time_period+1
    % update linear and angular velocity of robot
    V = (vr(k) + vl(k))/2;
    omega = (vr(k) - vl(k))/W;  
    
    % calculate next position    
    th(k+1) = th(k) + omega       *dt;    
    x(k+1)  = x(k)  + V*cos(th(k))*dt;    
    y(k+1)  = y(k)  + V*sin(th(k))*dt;
  end
  
end