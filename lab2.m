
%% Lab2 start robot
robot = neato('milli');
%% Lab2 warmup 1
robot.startLaser();
pause(3);
tic;
a = toc;
index = 1:360;
min_range = 2;
while (a<60)
    pause(0.5);
    ranges = transpose(robot.laser.LatestMessage.Ranges);
    x = 1:360;
    y = 1:360;
    j = 1;
    %to eliminate reading greater than 1.5 and smaller than 0.006
    while (j<361)
        if (ranges(j) > 1.5 || ranges(j) <0.06)
            ranges(j) = 0;
        end
        if (j >90 && j<270)
            ranges(j) = 0;
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
%% warm up 2 test
robot.startLaser();
pause(3);
ranges = robot.laser.LatestMessage.Ranges;
angle = (1:360)*(pi/180);
cos_th = cos(angle);
sin_th = sin(angle);
x = ranges.*cos_th;
y = ranges.*sin_th;
plot(-y,x);

%% true warmup 2
