classdef rangeImage < handle
 %rangeImage Stores a 1D range image and provides related services.

    properties(Constant)
        maxUsefulRange = 1;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end

    properties(Access = public)
        rArray = [];
        thArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    methods(Access = public)

        function obj = rangeImage(ranges,skip,cleanFlag)
        % Constructs a rangeImage for the supplied data.
        % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.thArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.thArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.thArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end

        function removeBadPoints(obj)
        % takes all points above and below two range thresholds
        % out of the arrays. This is a convenience but the result
        % should not be used by any routine that expects the points
        % to be equally separated in angle. The operation is done
        % inline and removed data is deleted.
        r_array = obj.rArray;
        %figure(6);
        %plot(r_array);
        x_array = obj.xArray;
        y_array = obj.yArray;
        for i = 1:length(r_array)
            if (r_array(i) > obj.maxUsefulRange || ...
                r_array(i) < obj.minUsefulRange)
                r_array(i) = 0;
                x_array(i) = 0;
                y_array(i) = 0;
            end
        end
        obj.rArray = r_array;
        obj.xArray = x_array;
        obj.yArray = y_array;
        %figure(7);
        %plot(r_array);
        end

 function plotRvsTh(obj, maxRange)
 % plot the range image after removing all points exceeding
 % maxRange
 r_array = obj.rArray;
 th_array = obj.thArray;
 for i = 1: length(r_array)
     if (r_array(i) > maxRange)
         r_array(i) = 0;
     end
 end
 plot(th_array, r_array);
 end

 function plotXvsY(obj, maxRange)
 % plot the range image after removing all points exceeding
 % maxRange
 x_array = obj.xArray;
 y_array = obj.yArray;
 r_array = obj.rArray;
 for i = 1:length(r_array);
     if r_array(i) > maxRange
         x_array(i) = 0;
         y_array(i) = 0;
     end
 end
 plot(x_array, y_array);
 end
 
 function [middle,pose] = findObject(obj)   
    minErr = 100000;
    maxNum = 0;
    bestTh = 0;
    middle = 0;
    bestZero = 0;
    %maxLength = 0.128;
    maxLength = 0.1;
     obj.plotXvsY(1);
    
    for i = 1:359
        
        
        [err,num,th,rightEnd,leftEnd] = obj.findLineCandidate(i,maxLength);
        
        % ignore any midpoints that are 0 away
        %   these are midpoints that were out of the intended range
        if obj.rArray(i) == 0
            continue;
        end
        
        % if the length of the line at midpoint i is 0, ignore
        if num == 0
            continue;
        end
        %hold on
        %x = obj.xArray(i);
        %y = obj.yArray(i);
        %plot(x,y,'y+');
        %hold off
        % if the error of the current midpoint is less than or equal
        %    to the smallest error seen so far, consider this midpoint
        if (num + 10 >= maxNum)
            % check to see which line is longer (we want the longer line)
            thisZero = obj.emptyLength(i, rightEnd, leftEnd);
            if thisZero >= bestZero
                minErr = err;
                maxNum = num;
                bestTh = th;
                middle = i;
                bestX = obj.xArray(i);
                bestY = obj.yArray(i);
                bestZero = thisZero;
                pose = [bestX;bestY;bestTh];
                %DEBUG statements
                %fprintf('new best is %d\n', i);
                %fprintf('bestZero is %d\n', bestZero);
                hold on;
                
                %x = obj.xArray(middle);
                %y = obj.yArray(middle);
                %plot(x,y,'b+');
                hold off;
            end
        else
            % error is bigger than previous best, so ignore midpoint
            continue;
        end
        
    end
   
    hold on;
    obj.plotXvsY(1);
    x = obj.xArray(middle);
    y = obj.yArray(middle);
    plot(x,y,'r+');
    hold off;
    
 end
 
 function err = lineFitError(obj,leftEnd, rightEnd)
     x_array = obj.xArray;
     y_array = obj.yArray;
     r_array = obj.rArray;
     err = 0;
     line_v = [x_array(rightEnd)-x_array(leftEnd);...
               y_array(rightEnd)-y_array(leftEnd)];
     
     num = 0;
     for i = leftEnd:rightEnd
         if r_array(i) ~= 0;
             num = num + 1;
             left_v = [x_array(i)-x_array(leftEnd);...
                       y_array(i)-y_array(leftEnd)];
             th = acos(norm(line_v .* left_v)/norm(line_v)/norm(left_v));
             d = sin(th) * norm(left_v);
             if isnan(d)
                 d = 0;
             end
             err = err + d;             
         end
     end
     err = err/num;
 end
 
 function [err,num,th,rightEnd,leftEnd] = findLineCandidate(obj,middle,maxLen)
 % Find the longest sequence of pixels centered at pixel
 % ?middle? whose endpoints are separated by a length less
 % than the provided maximum. Return the line fit error, the
 % number of pixels participating, and the angle of
 % the line relative to the sensor.
 length = 0; %length of the whole line
 inc_i = middle; %search index to right
 dec_i = middle; %search index to left
 num = 0; %num of pixel on the line
 removeBadPoints(obj);
 x_array = obj.xArray;
 y_array = obj.yArray;
 r_array = obj.rArray;
 rightEnd = 0;
 leftEnd = 0;
 while length < maxLen
     %search left and right of the middle
     if r_array(inc_i) == 0
         break;
     else
         inc_i = obj.inc(inc_i);
         num = num + 1;
     end
     if r_array(dec_i) == 0
         break;
     else
         dec_i = obj.dec(dec_i);
         num = num + 1;
     end     
     
     rightEnd = inc_i;
     leftEnd  = dec_i;
     
     %find the left and right most distance to middle
     inc_x = double(x_array(inc_i));
     inc_y = double(y_array(inc_i));
     dec_x = double(x_array(dec_i));
     dec_y = double(y_array(dec_i));
     length = sqrt((inc_x - dec_x)^2 + (inc_y - dec_y)^2);
 end
 
   inc_i = obj.dec(inc_i);
   dec_i = obj.inc(dec_i);
   left_x = x_array(dec_i);
   left_y = y_array(dec_i);
   right_x = x_array(inc_i);
   right_y = y_array(inc_i);
   th = atan2(-double(right_x-left_x),double(right_y-left_y));

   err = obj.lineFitError(dec_i,inc_i);        
 end
 
 function zero_length = emptyLength(obj, middle, rightEnd, leftEnd)
     zero_length = 0;
     r_array = obj.rArray;
     inc_i = rightEnd;
     dec_i = leftEnd;
     
     % find actual edge
     while r_array(inc_i) ~= 0 
         inc_i = obj.inc(inc_i);
     end
     
     while r_array(dec_i) ~= 0
         dec_i = obj.inc(dec_i);
     end
     
     while zero_length < 360
         % search left and right of where we think the board is
         if r_array(inc_i) ~= 0
             break;
         else
             inc_i = obj.inc(inc_i);
             zero_length = zero_length + 1;
         end
         if r_array(dec_i) ~= 0
             break;
         else
             dec_i = obj.dec(dec_i);
             zero_length = zero_length + 1;
         end
         zero_length = zero_length + 1;
     end
 end

 % Modulo arithmetic on nonnegative integers. MATLABs choice to
 % have matrix indices start at 1 has implications for
 % calculations relating to the position of a number in the
 % matrix. In particular, if you want an operation defined on
 % the sequence of numbers 1 2 3 4 that wraps around, the
 % traditional modulus operations will not work correctly.
 % The trick is to convert the index to 0 1 2 3 4, do the
 % math, and convert back.

 function out = inc(obj,in)
 % increment with wraparound over natural numbers
 out = indexAdd(obj,in,1);
 end

 function out = dec(obj,in)
 % decrement with wraparound over natural numbers
 out = indexAdd(obj,in,-1);
 end

 function out = indexAdd(obj,a,b)
 % add with wraparound over natural numbers. First number
% ?a? is "natural" meaning it >=1. Second number is signed.
 % Convert a to 0:3 and add b (which is already 0:3).
 % Convert the result back by adding 1.
 out = mod((a-1)+b,obj.numPix)+1;
 end
 end
end