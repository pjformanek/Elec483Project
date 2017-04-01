function [pframe, MVframe] = ThreeStep(aframe,tframe,bsize,R)
% 3Step search block matching algorithm for motion compensation:
%  @arg
%       aframe: anchor/current frame
%       tframe: target/reference frame
%       bsize: block size
%       R:  search range, the total search area is ((2*R+1)^2)
%  @output
%       pframe: Predicted frame
%       MVframe:  Motion vectors, using multidimensional array
%              MVframe(:,:,1) are all the dx values
%              MVframe(:,:,2) are all the dy values
%  Produces the Motion Vector Field, the Residual Frame, and a Predicted 
%  Frame from the Motion Vector Field.

[height, width] = size(aframe);
aframe = double(aframe);
tframe = double(tframe);
wremain = mod(width,bsize);
hremain = mod(height,bsize);
wpads = 0;
hpads = 0;
% adds zero padding for integer division of frame into macroblocks
if(wremain ~= 0)
   wpads = bsize-wremain;
   aframe = padarray(aframe,[0,wpads],'post');
   tframe = padarray(tframe,[0,wpads],'post');
end
if(hremain ~= 0)
  hpads = bsize-hremain;
  aframe = padarray(aframe,[hpads,0],'post');
  tframe = padarray(tframe,[hpads,0],'post');
end
modifiedsize = size(aframe);
pframe = zeros(modifiedsize(1),modifiedsize(2));
MVframe = zeros(modifiedsize(1)/bsize,modifiedsize(2)/bsize,2);
for y = 1:bsize:height
    for x = 1:bsize:width
        % reset the search variables for each block
        MAD = 255; %max error possible
        minMAD = MAD; 
        range = R;
        mvx = 0;
        mvy = 0;
        xoffset = 0;
        yoffset = 0;

    % searches till the range is 1 and minimizes the error using MeanAbsDiff
    while (range > 1)  
    % reduce the range by half for each step
        range = ceil(range/2);
        % reset the search adjustments each step
        dx = 0;
        dy = 0;
        % check the center only once on first step
        if(range == ceil(R/2))
            MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                  - tframe(y:y+bsize-1,x:x+bsize-1))))/(bsize*bsize);
            minMAD = MAD;
        end
        for i = -range:range:range
            for j = -range:range:range
                % if the current search block is in the image bounds
                if(y+i+yoffset>0 && y+i+yoffset+bsize-1<height+1 &&...
                        x+j+xoffset>0 && x+j+xoffset+bsize-1<width+1 && ...
                        (i ~= 0 || j ~=  0)) % and not center block
                   % compare blocks between images
                  MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                          - tframe(y+i+yoffset:y+i+yoffset+bsize-1,...
                         x+j+xoffset:x+j+xoffset+bsize-1))))/(bsize*bsize);
               % if a block with a lower MAD is found, keep track of the
               % adjustments, and save new MAD value
                if (MAD < minMAD)
                    minMAD = MAD;
                    dx = j;
                    dy = i;
                end
                end
            end
        end
        % update the offsets and the motion vectors
        xoffset = xoffset + dx;
        yoffset = yoffset + dy;
        mvx = mvx + dx;
        mvy = mvy + dy;
    end
        % build up the motion vectors of the frame
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,1) = xoffset;
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,2) = yoffset;
        % predict the anchor frame from the target frame and motion vector 
        pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+yoffset:y+yoffset...
            +bsize-1, x+xoffset:x+xoffset+bsize-1);
    end
end
% removes the zero padding from macroblocking
pframe = pframe(1:end-hpads,1:end-wpads);
end