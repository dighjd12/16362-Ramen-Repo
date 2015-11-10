classdef lineMapLocalizer < handle
%mapLocalizer A class to match a range scan against a map in
% order to find the true location of the range scan relative to
% the map.
    properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
    end

    properties(Access = private)
    end

    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;
    end
    methods(Access=public)
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
        % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end
        function ro2 = closestSquaredDistanceToLines(obj,pi)
        % Find the squared shortest distance from pi to any line
        % segment in the supplied list of line segments.
        % pi is an array of 2d points
        % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
        % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = closestSquaredDistanceToLines(obj,worldPts);
            ids = find(sqrt(r2) > obj.maxErr);
        end
        function avgErr = fitError(obj,pose,ptsInModelFrame)
        % Find the standard deviation of perpendicular distances of
        % all points to all lines
        % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;

            r2 = closestSquaredDistanceToLines(obj,worldPts);
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
            % not enough points to make a guess
                avgErr = inf;
            end
        end
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
        % Computes the gradient of the error function

            errPlus0 = fitError(obj,poseIn,modelPts);

            eps = 0.001;
            dp = [eps ; 0.0 ; 0.0];
            newPose = pose(poseIn.getPoseVec+dp);
            newPose2 = pose(poseIn.getPoseVec+[0.0; eps; 0.0]);
            newPose3 = pose(poseIn.getPoseVec+[0.0; 0.0; eps]);
          
            d1 = (1/eps)*(fitError(obj,newPose,modelPts)-errPlus0);
            d2 = (1/eps)*(fitError(obj,newPose2,modelPts)-errPlus0);
            d3 = (1/eps)*(fitError(obj,newPose3,modelPts)-errPlus0);
            
            J = [d1 d2 d3];
            
            %fprintf('th jacobian: %d\n', d3);

        end
        function [success, outPose] = refinePose(obj,inPose,ptsInModelFrame,maxIters)
        % refine robot pose in world (inPose) based on lidar
        % registration. Terminates if maxIters iterations is
        % exceeded or if insufficient points match the lines.
        % Even if the minimum is not found, outPose will contain 
        % any changes that reduced the fit error. Pose changes that
        % increase fit error are not included and termination
        % occurs thereafter.
            
            inPose = pose(robotModel.senToWorld(inPose));
        
            if size(ptsInModelFrame,2)<obj.minPts
                success=0;
                outPose=inPose;
                outPose= pose(robotModel.robToWorld(outPose));
                return;
            end %should this be after we throw outliers?
        
            %add tracker algorithm?
            ids = throwOutliers(obj,inPose,ptsInModelFrame);
            ptsInModelFrame(:,ids) = [];
            
            numIters = 0;
            [errPlus0,J] = getJacobian(obj,inPose,ptsInModelFrame);
            poseVec = inPose.getPoseVec();
            
            lastPoseVec = poseVec;
            lastErrPlus0 = errPlus0;
            
%             if numIters>maxIters
%                 disp('case1');
%             end
%             if sqrt(sum(J.^2))<=obj.gradThresh
%                 disp('case2');
%             end
%             if errPlus0<=obj.errThresh
%                 disp('case3');
%             end
            
            while numIters<=maxIters && sqrt(sum(J.^2))>obj.gradThresh && errPlus0>obj.errThresh
                
                newPose = pose(poseVec);
                [errPlus0,J] = getJacobian(obj,newPose,ptsInModelFrame);
              %  fprintf('in iteration: %d, err is: %d\n', numIters, errPlus0);
                if errPlus0>lastErrPlus0
                    poseVec = lastPoseVec;
                    break;
                end
                
                deltP = -obj.gain*J;
                poseVec = poseVec+deltP';
               % fprintf('th offset: %d\n', deltP(3));
                
               % fprintf('deltP: %d, %d, %d\n', deltP(1),deltP(2),deltP(3));
                
                numIters = numIters+1;
%               worldPts = newPose.bToA()*ptsInModelFrame;
%                 %%%%%%%%%plotting%%%%%%%%%%
%                 figure(1)
%                 hold on
%                 plot(worldPts(1,:),worldPts(2,:),'-mx'); %model points in world frame
%                 for i=1:length(obj.lines_p1)
%                     plot([obj.lines_p1(1,i) obj.lines_p2(1,i)],[obj.lines_p1(2,i) obj.lines_p2(2,i)], '-r');
%                 end %walls
%                 plot(poseVec(1),poseVec(2),'-xb');
%                 plot([poseVec(1),J(1)],[poseVec(2),J(2)],'-b');
%                 hold off
%                 %%%%%%%%%plotting%%%%%%%%%%
                
                lastPoseVec = poseVec;
                lastErrPlus0 = errPlus0;
                
               % pause(0.5);
            end
            success=1;
            outPose=pose(poseVec(1,1), poseVec(2,1), poseVec(3,1));
            outPose= pose(robotModel.robToWorld(outPose));
            
        end
    end
end