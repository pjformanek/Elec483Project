function [pframes] = NewThreeStep(aframes,tframes,bsize)
% New 3-Step search block matching algorithm for motion compensation:
%  @arg
%       aframe: anchor/current RGB frames 
%       tframe: target/reference RGB frames
%       bsize: block size
%      
%  @output
%       pframes: Predicted RGB frames

aframe = double(aframes(:,:,1));
pframes = zeros(size(aframes));
tframe = double(tframes(:,:,1));
[height, width] = size(aframe);
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
% preallocate for modified frames with padding
Rpframe = zeros(modifiedsize(1),modifiedsize(2));
Gpframe = zeros(modifiedsize(1),modifiedsize(2));
Bpframe = zeros(modifiedsize(1),modifiedsize(2));
Rtframe = zeros(modifiedsize(1),modifiedsize(2));
Gtframe = zeros(modifiedsize(1),modifiedsize(2));
Btframe = zeros(modifiedsize(1),modifiedsize(2));
Rtframe(1:height,1:width) = tframes(:,:,1);
Gtframe(1:height,1:width) = tframes(:,:,2);
Btframe(1:height,1:width) = tframes(:,:,3);
% for every macro block in the image a search is made
for y = 1:bsize:height
    for x = 1:bsize:width
        % reset the search variables for each macro block
        MAD = 255; % max error possible
        minMAD = MAD; % set the reference error to worst case
        range = 4;
        xoffset = 0;
        yoffset = 0;
% search 8 edge blocks at range 1/2 bsize and 1 center first
for i = -range:range:range
    for j = -range:range:range
        % if the current search block is in the image bounds
        if((y+i>0) && (y+i+bsize-1<height+1) && ...
                (x+j>0) && (x+j+bsize-1<width+1))
           % compare blocks between frames
          MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                  - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1)))) ...
                  /(bsize*bsize);
%        if a block with a lower MAD is found, keep track of the
       % adjustments, and save new MAD value
        if (MAD < minMAD)
            minMAD = MAD;
            xoffset = j;
            yoffset = i;
        end
        end
    end
end

% search 8 blocks adjacent to the center block
for i = -1:1
    for j = -1:1
        % if the current search block is in the image bounds
        if((y+i>0) && (y+i+bsize-1<height+1) && ...
                (x+j>0) && (x+j+bsize-1<width+1) && (i ~= 0 || j ~= 0))
           % compare blocks between frames
          MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                  - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1)))) ...
                  /(bsize*bsize);
%        if a block with a lower MAD is found, keep track of the
       % adjustments, and save new MAD value
        if (MAD < minMAD)
            minMAD = MAD;
            xoffset = j;
            yoffset = i;
        end
        end
    end
end

% if center block is best match, end search and go to next macro block
if(xoffset == 0 && yoffset ==0)
       % build up the motion vectors of the frame
%     mvframe((y-1)/bsize+1,(x-1)/bsize+1,1) = xoffset;
%     mvframe((y-1)/bsize+1,(x-1)/bsize+1,2) = yoffset;
    % predict the anchor frame from the target frame and motion vector 
    Rpframe(y:y+bsize-1,x:x+bsize-1) = Rtframe(y:y+bsize-1,x:x+bsize-1);
    Gpframe(y:y+bsize-1,x:x+bsize-1) = Gtframe(y:y+bsize-1,x:x+bsize-1);
    Bpframe(y:y+bsize-1,x:x+bsize-1) = Btframe(y:y+bsize-1,x:x+bsize-1);
    continue
    
% if best match is at edge of range continue with normal 3 step method
elseif(abs(xoffset) == 4 || abs(yoffset) == 4)
    % search 2 more steps
    while (range > 1)  
    % reduce the range by half for each step
        range = range/2;
        % reset the search adjustments each step
        dx = 0;
        dy = 0;
        % search 8 of the edge blocks
        for i = -range:range:range
            for j = -range:range:range
                % if the current search block is in the image bounds
                if(y+i+yoffset>0 && y+i+yoffset+bsize-1<height+1 &&...
                        x+j+xoffset>0 && x+j+xoffset+bsize-1<width+1 && ...
                        (i ~= 0 || j ~= 0))
                   % compare blocks between frames
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
        % update the offsets
        xoffset = xoffset + dx;
        yoffset = yoffset + dy;
    end
    
% if best match is adjacent to the target block
else
    dx = 0;
    dy = 0;
    
    % search 3 blocks
    if xoffset == 0
        for j = -1:1
         % if the current search block is in the image bounds
            if(y+2*yoffset>0 && y+2*yoffset+bsize-1<height+1 && ...
                    x+j>0 && x+j+bsize-1<width+1)
               % compare blocks between frames
                MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                      - tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
                     x+j:x+j+bsize-1))))/(bsize*bsize);
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
                if (MAD < minMAD)
                    minMAD = MAD;
                    dx = j;
                    dy = 2*yoffset;
                end
            end
        end
        % update the offsets
        xoffset = xoffset + dx;
        yoffset = yoffset + dy;
    
    % search 3 blocks
    elseif yoffset == 0
         for i = -1:1
         % if the current search block is in the image bounds
            if(y+i>0 && y+i+bsize-1<height+1 && ...
                    x+2*xoffset>0 && x+2*xoffset+bsize-1<width+1)
               % compare blocks between frames
                MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                      - tframe(y+i:y+i+bsize-1,...
                     x+2*xoffset:x+2*xoffset+bsize-1))))/(bsize*bsize);
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
                if (MAD < minMAD)
                    minMAD = MAD;
                    dx = 2*xoffset;
                    dy = i;
                end
            end
        end
        % update the offsets
        xoffset = xoffset + dx;
        yoffset = yoffset + dy;
        
   %search 5 blocks associated with the offset corner block
    else
        % if the current search block is in the image bounds
        if(y+2*yoffset>0 && y+2*yoffset+bsize-1<height+1 && ...
                x+2*xoffset>0 && x+2*xoffset+bsize-1<width+1)
           % compare blocks between frames
            MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                  - tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
                 x+2*xoffset:x+2*xoffset+bsize-1))))/(bsize*bsize);
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
            if (MAD < minMAD)
                minMAD = MAD;
                dx = 2*xoffset;
                dy = 2*yoffset;
            end
        end
        for j = 1:2
             % if the current search block is in the image bounds
            if(y+2*yoffset>0 && y+2*yoffset+bsize-1<height+1 && ...
                    x+j+2*xoffset>0 && x+j+2*xoffset+bsize-1<width+1)
               % compare blocks between frames
                MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                      - tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
                    x+j+2*xoffset:x+j+2*xoffset+bsize-1))))/(bsize*bsize);
               % if a block with a lower MAD is found, keep track of the
               % adjustments, and save new MAD value
                if (MAD < minMAD)
                    minMAD = MAD;
                    dx = 2*xoffset+j;
                    dy = 2*yoffset;
                end
            end
        end
        for i = 1:2
             % if the current search block is in the image bounds
            if(y+i+2*yoffset>0 && y+i+2*yoffset+bsize-1<height+1 && ...
                    x+2*xoffset>0 && x+2*xoffset+bsize-1<width+1)
               % compare blocks between frames
                MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                      - tframe(y+i+2*yoffset:y+i+2*yoffset+bsize-1,...
                    x+2*xoffset:x+2*xoffset+bsize-1))))/(bsize*bsize);
              % if a block with a lower MAD is found, keep track of the
              % adjustments, and save new MAD value
                if (MAD < minMAD)
                    minMAD = MAD;
                    dx = 2*xoffset+j;
                    dy = 2*yoffset;
                end
            end
        end
        xoffset = dx;
        yoffset = dy;
    end
end
    % build up the motion vectors of the frame
%     mvframe((y-1)/bsize+1,(x-1)/bsize+1,1) = xoffset;
%     mvframe((y-1)/bsize+1,(x-1)/bsize+1,2) = yoffset;
    % predict the anchor frame from the target frame and motion vector 
   Rpframe(y:y+bsize-1,x:x+bsize-1) = Rtframe(y+yoffset:y+yoffset+bsize-1,...
       x+xoffset:x+xoffset+bsize-1);
   Gpframe(y:y+bsize-1,x:x+bsize-1) = Gtframe(y+yoffset:y+yoffset+bsize-1,...
       x+xoffset:x+xoffset+bsize-1);
   Bpframe(y:y+bsize-1,x:x+bsize-1) = Btframe(y+yoffset:y+yoffset+bsize-1,...
       x+xoffset:x+xoffset+bsize-1);
end
end
% removes the zero padding from macroblocking
pframes(:,:,1) = Rpframe(1:end-hpads,1:end-wpads);
pframes(:,:,2) = Gpframe(1:end-hpads,1:end-wpads);
pframes(:,:,3) = Bpframe(1:end-hpads,1:end-wpads);
end