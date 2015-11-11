classdef stateEstimator < handle
    
    properties(Access = public)
    
        lmLocalizer;
        worldLineArray;
        lidarSkip;
        
        poseFused;
        startPose;
        k;
        
        plidarArray;
        i;
    end
    
    methods(Static = true)
        function obj = stateEstimator(lmLocalizer, worldLineArray, lidarSkip)
            
            obj.lmLocalizer = lmLocalizer;
            obj.worldLineArray = worldLineArray;
            obj.lidarSkip = lidarSkip;
            
            obj.plidarArray = zeros(3,1);
            obj.i = 2;
            obj.k = 0.15;
            obj.poseFused = [0;0;0]; %posefused in robot start frame
        end
        function setInitPose(obj,initPose) 
            obj.poseFused = initPose;
            obj.startPose = initPose;
        end
        function pf = fusePose(obj, robot, dx, dy, dth, rsTowM)
            
            obj.processOdometryData(obj, dx, dy, dth);
            
            %pose fused is in rs frame
            modelPts = obj.processRangeImage(obj, robot);
            maxIters = 50;
            
            pf_w = pose.matToPoseVec(rsTowM * pose(obj.poseFused).bToA());
            
            p = pose(pf_w);
          %  fprintf('posefused is [%d; %d; %d;]', obj.poseFused(1),obj.poseFused(2),obj.poseFused(3));
            [success, outPose] = refinePose(obj.lmLocalizer,p,modelPts,maxIters);
    
            if(success)
                poseLidar = outPose.getPoseVec(); %world frame
                obj.plidarArray(:, obj.i) = poseLidar;
                
                pf_w(1,1) = pf_w(1,1) + obj.k*(poseLidar(1,1)-pf_w(1,1)); %change x
                pf_w(2,1) = pf_w(2,1) + obj.k*(poseLidar(2,1)-pf_w(2,1)); %change y
                th2 = poseLidar(3,1);
                th1 = pf_w(3,1);
                pf_w(3,1) = pf_w(3,1) + obj.k*(atan2(sin(th2-th1),cos(th2-th1)));
            else
                obj.plidarArray(:, obj.i) = [0;0;0];
            end
            obj.i= obj.i+1;
            
            obj.poseFused = pose.matToPoseVec( (rsTowM^-1) * pose(pf_w).bToA());
            pf = obj.poseFused;
     
        end
        
        function processOdometryData(obj, dx, dy, dth)
            
            obj.poseFused = obj.poseFused + [dx;dy;dth];
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