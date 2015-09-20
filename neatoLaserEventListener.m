function neatoLaserEventListener(obj,msg)

          global neatoLaserDataTimeStamp;
          global neatoLaserRanges;
      %    ...% Do some stuff
          neatoLaserDataTimeStamp=obj.LatestMessage.Header.Stamp.Sec +(obj.LatestMessage.Header.Stamp.Nsec/1000000000);
          neatoLaserRanges=obj.LatestMessage.Ranges
          %The following line is equivalent to the previous line.
      %    neatoLaserRanges=msg.Ranges
       %   ...%Do some stuff
  end