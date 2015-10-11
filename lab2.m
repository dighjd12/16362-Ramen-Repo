
%% Lab2 start robot
robot = neato('yotta');
%% Lab2 warmup 2
robot.startLaser();
pause(3);
tic;
a = toc;
while (a<20)
    pause(0.5);
    ranges = transpose(robot.laser.LatestMessage.Ranges);
    x = 1:360;
    y = 1:360;
    j = 1;
    %to eliminate reading greater than 1.5 and smaller than 0.006
    while (j<361)
        if (ranges(j) > 1.5 || ranges(j) <0.06)
            ranges(j) = 0; % in the right range
        end
        if (j >90 && j<270)
            ranges(j) = 0; % abs(bearing)< maxBearing
        end
        [x_i,y_i,th_i] = irToXy(j,ranges(j));
        x(j) = x_i;
        y(j) = y_i;
        j = j+1;
        
    end
    
    plot(-y,x,'re.');
    axis([-0.5 0.5 -0.5 0.5]);
    a = toc;
end
disp('quit');
robot.stopLaser();

%% warm up 3

robot.startLaser();
pause(3);
tic;
a = toc;
idealObjectRange = 1;

while (a<40)
    pause(0.5);
    ranges = transpose(robot.laser.LatestMessage.Ranges);
    x = 1:360;
    y = 1:360;
    j = 1;
    
    min_range = 2;
    min_index = 0;
    %to eliminate reading greater than 1.5 and smaller than 0.006
    while (j<361)
        if (ranges(j) > 1.5 || ranges(j) <0.06)
            ranges(j) = 0; % in the right range
        end
        if (j >90 && j<270)
            ranges(j) = 0; % abs(bearing)< maxBearing
        end
        if (ranges(j)<min_range && ranges(j)~=0)
            min_range = ranges(j);
            min_index = j;
        end
        
        [x_i,y_i,th_i] = irToXy(j,ranges(j));
        x(j) = x_i;
        y(j) = y_i;
        j = j+1;
    end
    
    if(min_index~=0) 
        if(min_range>idealObjectRange)
            robot.sendVelocity(0.02, 0.02);
        end
        if(min_range<idealObjectRange)
            robot.sendVelocity(-0.02, -0.02);
        end
    end
        
    plot(-y,x,'re.');
    axis([-2 2 -2 2]);
    a = toc;
end
disp('quit');
robot.sendVelocity(0, 0);
robot.stopLaser();

%%

%means something is within the range
      %  if(min_range>idealObjectRange || min_range<idealObjectRange)
           % vel = round((min_range-idealObjectRange)*0.1, 2);
           % robot.sendVelocity(0.01, 0.01);
         %   disp(vel);
        %end

%% Challenge Task

robot.startLaser();
pause(3);
tic;
a = toc;
idealObjectRange = 1;
dt = 0;
dth = 0;
lastT = 0;
lastV = 0;
lastAngle = 0;

while (a<20)
    pause(0.5);
    ranges = transpose(robot.laser.LatestMessage.Ranges);
    j = 1;
    min_range = 2;
    min_index = 0;
    %to eliminate reading greater than 1.5 and smaller than 0.006
    while (j<361)
        if (~(j >90 && j<270) && ~(ranges(j) > 1.5 || ranges(j) <0.06))
            if (ranges(j)<min_range && ranges(j)~=0)
            min_range = ranges(j);
            min_index = j;
            end
        end
        j = j+1;
    end
    
    if(min_index~=0) %some object to follow
        if(min_range>idealObjectRange)
            thisT = toc;
            dt = thisT - lastT;
            %ds = lastV*dt;
            dth = mod((min_index - lastAngle),360);
            w = dth./dt;
            vr = 0.01 + .25/2*w;
            vl = 0.01 - .25/2*w;
            robot.sendVelocity(vl, vr);
            lastV = (vl+vr)./2;
            lastAngle = min_index;
        end
        if(min_range<idealObjectRange)
            thisT = toc;
            dt = thisT - lastT;
            %ds = lastV*dt;
            dth = mod((min_index - lastAngle),360);
            w = dth./dt;
            vr = -0.01+ 0.25/2*w;
            vl = -0.01 - 0.25/2*w;
            robot.sendVelocity(vl, vr);
            lastV = (vl+vr)./2;
            lastAngle = min_index;
        end
    end
    
    [x_i,y_i,th_i] = irToXy(min_index, min_range);
        
    plot(-y_i,x_i,'x');
    axis([-2 2 -2 2]);
    a = toc;
    lastT = a;
end
disp('quit');
robot.sendVelocity(0, 0);
robot.stopLaser();

%% Challenge Task

robot.startLaser();
pause(3);
tic;
a = toc;
idealObjectRange = 1;
linV = 0.15;
robot_width = .25;
error = 0.01;

while (a<60)
    pause(0.2);
    ranges = transpose(robot.laser.LatestMessage.Ranges);
    j = 1;
    min_range = 2;
    min_index = 0;
    %to find the 'nearest object'
    while (j<361)
        if (~(j >90 && j<270) && ~(ranges(j) > 1.5 || ranges(j) <0.06)) %if j qualifies to be 'nearest object'
            if (ranges(j)<min_range && ranges(j)~=0) %if j is the new minimum range
            min_range = ranges(j);
            min_index = j;
            end
        end
        j = j+1;
    end
    
    [x_i,y_i,th_i] = irToXy(min_index, min_range);
    plot(-y_i,x_i,'x');
    axis([-2 2 -2 2]);
    
    if(min_index~=0) %indicates there is some object to follow
        if(min_range>idealObjectRange + error)
            w = double(robot_width.*linV.*(sin(th_i))/min_range);
            vr = linV+w;
            vl = linV-w;
            disp('going forward with');
           % disp(w); %positive if object is to the right and negative if left
            %disp(th_i);
            disp(vl);
            disp(vr);
            robot.sendVelocity(vl, vr);
            pause(0.001);
        end
        if(min_range<idealObjectRange - error)
            w = double(robot_width.*-linV.*(sin(th_i))/min_range);
            vl = -(linV - w);
            vr = -(linV + w);
            disp('going backward with ');
         %   disp(w);
          %  disp(th_i);
            disp(vl);
            disp(vr);
            robot.sendVelocity(vl, vr);
            pause(0.001);
        end
        if(~(min_range<idealObjectRange - error || min_range>idealObjectRange + error))
            disp('stopping');
            robot.sendVelocity(0, 0);
            pause(0.001);
        end
    end
        
    a = toc;
end
disp('quit');
robot.sendVelocity(0, 0);
robot.stopLaser();

