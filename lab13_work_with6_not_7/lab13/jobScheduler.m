classdef jobScheduler
    properties
        dropLocationList;
        pickUpLocationList;
        robotPose;
    end
    methods(Static = true)
        function obj = jobScheduler(dropLocationList,pickUpLocationList,robotPose)
            obj.dropLocationList = dropLocationList;
            obj.pickUpLocationList = pickUpLocationList;
            obj.robotPose = robotPose;        
        end
        function scheduleJob(obj,robot,mrpl)
            mrpl.turnRelAngle(mrpl,robot,pi(),0);
            i = 1;
            j = 1;
            while j <= length(obj.dropLocationList)
                %pause(1);
                pickPose = obj.pickUpLocationList(i,:);
                dropPose = obj.dropLocationList(j,:);
                success = mrpl.pickDropObject(mrpl,robot,pickPose);
                if success
                robot.sendVelocity(0,0);
                robot.forksUp();
                mrpl.turnRelAngle(mrpl,robot,pi(),0);
                reading = transpose(double(robot.laser.LatestMessage.Ranges));
                image = rangeImage(reading,1,0); 
                onBoard = image.isItOnBoard;
                
                if onBoard
                    %pause(1);
                    mrpl.executeTrajectorySE(mrpl,robot,dropPose(1),dropPose(2),...
                                             dropPose(3),1); 
                    %pause(1);
                    robot.forksDown();
                    mrpl.moveRelDistance(mrpl,robot,0.1,0);
                    %pause(1);
                    mrpl.turnRelAngle(mrpl,robot,pi(),0);
                    j = j+1;
                    i = i+1;
                else
                    %check if the backup amount is enough!!!
                    mrpl.moveRelDistance(mrpl,robot,0.7,-1);
                    robot.forksDown();
                    mrpl.turnRelAngle(mrpl,robot,pi(),0);   
                    i = i + 1;
                end
                else
                    i = i + 1;
                end
                
            end
        end
    end
end