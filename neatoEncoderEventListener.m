function neatoEncoderEventListener (obj, msg)

        global neatoEncoderDataTimestamp;
        global neatoEncoderLeft;
        global neatoEncoderRight;
       % global neatoEncoderFrame;

    %persistent neatoEncoderFrame;
        
        
       % global leftArray;
        global timeArray;
        global leftArray;
        global rightArray;
        global vlSoFar;
        global vrSoFar;
        
        %initialize
    %    if isempty(neatoEncoderFrame)
     %       neatoEncoderFrame = 0;
      %  end
        
        neatoEncoderDataTimestamp = obj.LatestMessage.Header.Stamp.Sec +(obj.LatestMessage.Header.Stamp.Nsec/1000000000);
        neatoEncoderLeft = obj.LatestMessage.Left;
        neatoEncoderRight = obj.LatestMessage.Right;
     %   neatoEncoderFrame = neatoEncoderFrame + 1;
        
        if(isempty(timeArray))
            timeArray = [timeArray, neatoEncoderDataTimestamp];
            leftArray = [leftArray, neatoEncoderLeft];
            rightArray = [rightArray, neatoEncoderRight];
            vlSoFar = zeros(1);
            vrSoFar = zeros(1);
        end
        
        if timeArray(length(timeArray))~=neatoEncoderDataTimestamp
            %increase timeArray, leftArray, rightArray, and compute vl vr
             vlFeedback = (neatoEncoderLeft - leftArray(length(leftArray)))./(neatoEncoderDataTimestamp - timeArray(length(timeArray)));
            vrFeedback = (neatoEncoderRight - rightArray(length(rightArray)))./(neatoEncoderDataTimestamp - timeArray(length(timeArray)));
        
            timeArray = [timeArray, neatoEncoderDataTimestamp];
            leftArray = [leftArray, neatoEncoderLeft];
           
            vlSoFar = [vlSoFar, vlFeedback];
             vrSoFar = [vrSoFar, vrFeedback];
        end
   

end

