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
 

 
 function [err,num,th] = findLineCandidate(obj,middle,maxLen)
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
 
 

 function num = numPixels(obj)
 num = obj.numPix;
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