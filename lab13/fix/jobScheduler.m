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
            for i = 1:length(obj.pickUpLocationList)
                pause(1);
                pickPose = obj.pickUpLocationList(i,:);
                dropPose = obj.dropLocationList(i,:);
                mrpl.pickDropObject(mrpl,robot,pickPose);
                robot.sendVelocity(0,0);
                robot.forksUp();
                pause(1);
                mrpl.turnRelAngle(mrpl,robot,pi(),0);
                pause(1);
                mrpl.executeTrajectorySE(mrpl,robot,dropPose(1),dropPose(2),...
                                         dropPose(3),1); 
                pause(1);
                robot.forksDown();
                mrpl.moveRelDistance(mrpl,robot,0.1,0);
                pause(1);
                mrpl.turnRelAngle(mrpl,robot,pi(),0);
            end
        end
    end
end