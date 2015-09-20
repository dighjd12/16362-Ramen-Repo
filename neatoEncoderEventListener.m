function neatoEncoderEventListener (obj, msg)

        global neatoEncoderDataTimestamp;
        global neatoEncoderLeft;
    %    global neatoEncoderRight;
       % global neatoEncoderFrame;

    %persistent neatoEncoderFrame;
        
        
       % global leftArray;
        global timeArray;
        global leftArray;
        
        %initialize
    %    if isempty(neatoEncoderFrame)
     %       neatoEncoderFrame = 0;
      %  end
        
        neatoEncoderDataTimestamp = obj.LatestMessage.Header.Stamp.Sec +(obj.LatestMessage.Header.Stamp.Nsec/1000000000);
        neatoEncoderLeft = obj.LatestMessage.Left
    %    neatoEncoderRight = obj.LatestMessage.Right;
     %   neatoEncoderFrame = neatoEncoderFrame + 1;
        timeArray = [timeArray, neatoEncoderDataTimestamp];
        leftArray = [leftArray, neatoEncoderLeft];

end

