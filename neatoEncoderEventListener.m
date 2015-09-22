function neatoEncoderEventListener (obj, msg)

        global neatoEncoderDataTimestamp;
        global neatoEncoderLeft;
        global neatoEncoderRight;
     %   global neatoEncoderFrame;

    %persistent neatoEncoderFrame;
        global vlFeedback;
        global vrFeedback;
        
       % global leftArray;
        global timeArray;
        global leftArray;
        global rightArray;
        global vlSoFar;
        global vrSoFar;
        
        global timeStamp;
        global nTimeStamp;
        
        global timeDiff;
        
        %initialize
    %    if isempty(neatoEncoderFrame)
     %       neatoEncoderFrame = 0;
      %  end
        timeStamp = double(obj.LatestMessage.Header.Stamp.Sec);
        nTimeStamp = double((double(obj.LatestMessage.Header.Stamp.Nsec)/(10^9)));
        
        neatoEncoderDataTimestamp = double(obj.LatestMessage.Header.Stamp.Sec) +double((double(obj.LatestMessage.Header.Stamp.Nsec)/(10^9)));
           %in s
        neatoEncoderLeft = double(obj.LatestMessage.Left);
        neatoEncoderRight = double(obj.LatestMessage.Right);
     %   neatoEncoderFrame = neatoEncoderFrame + 1;
        
        if(isempty(timeArray))
            timeArray = [timeArray, neatoEncoderDataTimestamp];
            leftArray = [leftArray, neatoEncoderLeft];
            rightArray = [rightArray, neatoEncoderRight];
         %   neatoEncoderFrame =0;
            vlSoFar = zeros(1);
            vrSoFar = zeros(1);
        end
        
        if timeArray(length(timeArray))~=neatoEncoderDataTimestamp
            %increase timeArray, leftArray, rightArray, and compute vl vr
            
            timeArray = [timeArray, neatoEncoderDataTimestamp];
            leftArray = [leftArray, neatoEncoderLeft];
            rightArray = [rightArray, neatoEncoderRight];
            
            timeDiff = (timeArray(length(timeArray)) - timeArray(length(timeArray)-1));
            timeDiff = double(timeDiff); %dt
            vlFeedback = (leftArray(length(leftArray)) - leftArray(length(leftArray)-1))./timeDiff;
            vrFeedback = (rightArray(length(rightArray)) - rightArray(length(rightArray)-1))./timeDiff;
            vlFeedback = double(vlFeedback);
            vrFeedback = double(vrFeedback); %in mm/s
            
            vlSoFar = [vlSoFar, (vlFeedback)];
            vrSoFar = [vrSoFar, (vrFeedback)];
        end
   
      %  neatoEncoderFrame = neatoEncoderFrame +1;

      %  pause(0.005);
end

