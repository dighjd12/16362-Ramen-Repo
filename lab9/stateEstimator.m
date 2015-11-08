classdef stateEstimator < handle
    
    properties(Access = public)
    
        lmLocalizer;
        worldLineArray;
        lidarSkip;
        
        poseFused;
        k;
    end
    
    methods(Static = true)
        function obj = stateEstimator(lmLocalizer, worldLineArray, lidarSkip)
            
            obj.lmLocalizer = lmLocalizer;
            obj.worldLineArray = worldLineArray;
            obj.lidarSkip = lidarSkip;
            
            obj.k = 0.25;
        end
        function setInitPose(obj,initPose)
            obj.poseFused = initPose;
        end
        function pf = fusePose(obj, dx, dy, dth)
            
            processOdometryData(obj, dx, dy, dth);
            modelPts = processRangeImage();
            
            [success, outPose] = refinePose(obj.lmLocalizer,obj.poseFused,modelPts,maxIters);
    
            %fixing poseFused
            poseLidar = outPose;
            obj.poseFused(1,1) = obj.poseFused(1,1) + obj.k*(poseLidar(1,1)-obj.poseFused(1,1)); %change x
            obj.poseFused(2,1) = obj.poseFused(2,1) + obj.k*(poseLidar(2,1)-obj.poseFused(2,1)); %change y
            th2 = poseLidar(3,1);
            th1 = obj.poseFused(3,1);
            obj.poseFused(3,1) = obj.poseFused(3,1) + obj.k*(atan2(sin(th2-th1),cos(th2-th1)));

            pf = obj.poseFused;
            %%%%%%%%%%%%%%%%%%%%%%%%%%plotting below 
%             worldLidarPts = robotModel.senToWorld(obj.poseFused)*modelPts;
%             bodyPts1 = bToA(obj.poseFused)*robotModel.bodyGraph();
% 
%             %%%%%%%%%plotting%%%%%%%%%%
%             figure(1)
%             plot(obj.worldLineArray(1,:), obj.worldLineArray(2,:), '-b'); %plot walls
%             hold on
%             plot(bodyPts1(1,:),bodyPts1(2,:),'-g'); %robotPoints
%             plot(worldLidarPts(1,:), worldLidarPts(2,:), '-xr'); %plot lidar points in sensor frame
%             hold off
%             %%%%%%%%%plotting%%%%%%%%%%
         %   pause(0.001);
        end
        
        function processOdometryData(obj, dx, dy, dth)
            
            obj.poseFused = obj.poseFused + [dx;dy;dth];
        end
        function modelPts = processRangeImage(obj)
            ranges = transpose(double(robot.laser.LatestMessage.Ranges));
            
            rangePts = ranges(1:obj.lidarSkip:length(ranges));
            xPoints = zeros(1,length(rangePts));
            yPoints = zeros(1,length(rangePts));
            j=1;
            for n=1:length(rangePts)
                xPoints(n) = cosd(j)*rangePts(n);
                yPoints(n) = sind(j)*rangePts(n);
                j = j+obj.lidarSkip;
            end

            modelPts = [xPoints; yPoints; ones(1,length(xPoints))];
        end
        
    end
end