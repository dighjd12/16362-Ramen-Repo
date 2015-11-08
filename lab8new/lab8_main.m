%% changle task 
clc;
%tic;
%time = 0;
d = (0.20+0.038) * 0.5;
mrpl = mrplSystem();
%for n = 0 : 15
    %time = toc;
    disp('Starting Laser...');
    robot.startLaser();
    pause(5); % wait for laser data to come in
    reading = transpose(double(robot.laser.LatestMessage.Ranges));
    image = rangeImage(reading,1,1); 
    disp('Laser values read');
    
    % initialize values
    %   note that if we finish the loop with a middle of 0, this implies
    %   there are no possible midpoints
    minErr = 100000;
    maxNum = 0;
    bestTh = 0;
    middle = 0;
    
    disp('Finding Best Midpoint...');
    % loop through possible line midpoints
    for i = 1:360
        % use data from first midpoint as our first baseline
%         if i == 1;
%             [err,num,th] = image.findLineCandidate(i,0.125);
%             middle = i;
%             % if the line length is 0, the err will be NaN
%             % set this to something extremely high so it will be easy to
%             % 'beat'
%             if isnan(err)
%                 minErr = 1000;
%             else
%                 minErr = err;
%             end
%             maxNum = num;
%             bestTh = th;
%             %disp(['initializing values, error is: ', minErr]);
%             fprintf('initializing values, error is: %d', minErr);
%         end

        [err,num,th] = image.findLineCandidate(i,0.125);
        
                % ignore any midpoints that are 0 away
        %   these are midpoints that were out of the intended range
        if image.rArray(i) == 0
            continue;
        end
        
        
        
        % if the length of the line at midpoint i is 0, ignore
        if num == 0
            continue;
        end
        
        if i == 158
            i = i;
        end
        
        % if the error of the current midpoint is less than or equal
        %    to the smallest error seen so far, consider this midpoint
        if num >= maxNum
            % check to see which line is longer (we want the longer line)
            if maxNum <= num
                middle = i;
                % if the line length is 0, the err will be NaN
            % set this to something extremely high so it will be easy to
            % 'beat'
            if isnan(err)
                minErr = 1000;
            else
                minErr = err;
            end
                maxNum = num;
                bestTh = th;
                fprintf('new best midpoint found at:\n');
                fprintf('midpoint: %d \n', middle);
                fprintf('min error: %d \n', minErr);
                fprintf('max num: %d \n', maxNum);
                fprintf('theta: %d \n', bestTh);
                fprintf('\n');
            else
                % same error, but current line is smaller, so ignore
                continue;
            end
        else
            % error is bigger than previous best, so ignore midpoint
            continue;
        end
        
    end
    %disp(image.xArray(middle));
    %disp(image.yArray(middle));
    %disp('th');
    %disp(bestTh);
    hold on;
    image.plotXvsY(1);
    x = image.xArray(middle);
    y = image.yArray(middle);
    
    if y < 0
        y = y + 0.1;
    else
        y = y - 0.1;
    end
    plot(x,y,'r+');
    
    hold off;
    dx = cos(bestTh)*d;
    dy = sin(bestTh)*d;
    %dx2 = cos(bestTh)*0;
    %dy2 = sin(bestTh)*0;
    fprintf('calculating trajectory & driving');
    %if th > 0
    %    th = -pi+th;
    %else
    %    th = pi - th;
    %end
    
    mrpl.executeTrajectory(mrpl,robot,y+dy,x-dx,-bestTh,1);
    
%end
%error.('finished execution');
robot.stopLaser();



%% excercise 1 range image array
arraySize = 20; % real size is arraySize * 360
arrayCellSize = 360; % range scans 360 degree
large_range_image = zeros(1,arraySize * arrayCellSize); %large array asked
robot.startLaser(); %spin the laser
pause(5);
beep on
for i = 1:arraySize
    reading = double(transpose(robot.laser.LatestMessage.Ranges));
    disp(max(reading));
    large_range_image((i-1)*arrayCellSize+1:i*arrayCellSize) = reading;
    pause(1);
    beep;
end
beep;
beep;
beep;
robot.stopLaser();
save('range_image','large_range_image');

%% test excerise 2 rangeImage class
range = zeros(1,360);
range(300) = 5;
range(301) = 0.0001;
range(302) = 0.5;
image = rangeImage(range,1,1);
image.rArray(300);
image.rArray(301);
image.rArray(302);
image.thArray(360);
%% start up the robot
robot = neato('milli');
%% shut down the robot
robot.close();
%robot.shutdown(); %won't be needed for real robot
clear all;
close all;
beep on;
beep;

%%
th = -pi/2;
x = 0.3;
y = 0;
d = 0;
mrpl = mrplSystem();
dx = sin(th)*d;
dy = cos(th)*d;
mrpl.executeTrajectory(mrpl,robot,y-dy,-x+dx,th,1);

