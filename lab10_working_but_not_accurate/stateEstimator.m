classdef stateEstimator < handle
    
    properties(Access = public)
    
        lmLocalizer;
        worldLineArray;
        lidarSkip;
        
        poseFused;
        startPose;
        k;
    end
    
    methods(Static = true)
        function obj = stateEstimator(lmLocalizer, worldLineArray, lidarSkip)
            
            obj.lmLocalizer = lmLocalizer;
            obj.worldLineArray = worldLineArray;
            obj.lidarSkip = lidarSkip;
            
            obj.k = 0.05;
        end
        function setInitPose(obj,initPose)
            obj.poseFused = initPose;
            obj.startPose = initPose;
        end
        function pf = fusePose(obj, robot, x, y, th)
            
            obj.processOdometryData(obj, x, y, th);
            modelPts = obj.processRangeImage(obj, robot);
            maxIters = 20;
            
            
            p = pose(obj.poseFused);
            [success, outPose] = refinePose(obj.lmLocalizer,p,modelPts,maxIters);

    
            if(success)
            %fixing poseFused
            
                poseLidar = outPose.getPoseVec();
                poseLidar = poseLidar';
                
                x = obj.poseFused(1);
                y = obj.poseFused(2);
                th = obj.poseFused(3);
                
                obj.poseFused = [x + poseLidar(1)*obj.k;...
                                 y + poseLidar(2)*obj.k;...
                                 atan2(cos(th*(1)+poseLidar(3)*obj.k),...
                                       sin(th*(1)+poseLidar(3)*obj.k))];
                
                % uncomment to plot Lidar-only and Fused poses.
                % Uncommented because it slows down execution.
                %hold on;
                %figure(2);
                %plot(poseLidar(1), poseLidar(2), 'c+');
                %plot(obj.poseFused(1), obj.poseFused(2), 'y+');
                %hold off;

               
            end
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
        
        function processOdometryData(obj, x, y, th)            
            obj.poseFused = [x,y,th];
        end
        function modelPts = processRangeImage(obj, robot)
            
            ranges = transpose(double(robot.laser.LatestMessage.Ranges));
            rangePts = ranges(1:obj.lidarSkip:length(ranges));
            
            %disp(rangePts);
            if max(rangePts)==0
                error('lidar reading not working');
            end
            
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