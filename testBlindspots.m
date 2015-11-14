robot.startLaser();

n = 10;
ranges = zeros(n,360);

for i = 1:n
    pause(5);
    range = transpose(double(robot.laser.LatestMessage.Ranges));
    
    ranges(i,:) = range;
end
robot.stopLaser();

ranges_zeros = 0;
for i = 1:n
    ranges_zeros = [ranges_zeros find(~ranges(i,:))];
end