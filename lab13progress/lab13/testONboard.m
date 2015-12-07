%% test 



while(1)
    reading = transpose(double(robot.laser.LatestMessage.Ranges));
    image = rangeImage(reading,1,0); 
    onBoard = image.isItOnBoard;
    
    disp('******************');
    disp(onBoard);
    disp('******************');
    pause(0.1);
end
