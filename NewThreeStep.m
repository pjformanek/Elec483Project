function [pframe, mvframe] = NewThreeStep(aframe,tframe,bsize)
% New 3-Step search block matching algorithm for motion compensation:
%  @arg
%       aframe: anchor/current frame
%       tframe: target/reference frame
%       bsize: block size
%      
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
% adds zero padding for integer division of frame into macroblocks
if((wremain ~= 0) || (hremain ~=0))
   aframe = padarray(aframe,[hremain,wremain],'post');
   tframe = padarray(tframe,[hremain,wremain],'post');
end
modifiedsize = size(aframe);
pframe = zeros(modifiedsize(1),modifiedsize(2));
mvframe = zeros(modifiedsize(1)/bsize,modifiedsize(2)/bsize,2);
% for every macro block in the image a search is made
for y = 1:bsize:height
    for x = 1:bsize:width
        % reset the search variables for each macro block
        MAD = 255; % max error possible
        minMAD = MAD; % set the reference error to worst case
        range = 4;
        xoffset = 0;
        yoffset = 0;
%         ablock = uint8(aframe(y:y+bsize-1,x:x+bsize-1)); % used for debug
%         subplot(1,3,1),imshow(ablock);  % used for debug
%         title('target block'); % for debug

% search 8 edge blocks at range 4 and 1 center first
for i = -range:range:range
    for j = -range:range:range
        % if the current search block is in the image bounds
        if((y+i>0) && (y+i+bsize-1<height+1) && ...
                (x+j>0) && (x+j+bsize-1<width+1))
           % compare blocks between frames
          MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                  - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1)))) ...
                  /(bsize*bsize);
%                 tblock = uint8(tframe(y+i:y+i+bsize-1,...
%                     x+j:x+j+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
%        if a block with a lower MAD is found, keep track of the
       % adjustments, and save new MAD value
        if (MAD < minMAD)
%                 subplot(1,3,3),imshow(tblock);   % for debug                 
%                 title('best block'); % for debug
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
%                 tblock = uint8(tframe(y+i:y+i+bsize-1,...
%                     x+j:x+j+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
%        if a block with a lower MAD is found, keep track of the
       % adjustments, and save new MAD value
        if (MAD < minMAD)
%                 subplot(1,3,3),imshow(tblock);   % for debug                 
%                 title('best block'); % for debug
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
    mvframe((y-1)/bsize+1,(x-1)/bsize+1,1) = xoffset;
    mvframe((y-1)/bsize+1,(x-1)/bsize+1,2) = yoffset;
    % predict the anchor frame from the target frame and motion vector 
   pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+yoffset:y+yoffset+bsize-1,...
        x+xoffset:x+xoffset+bsize-1);
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
%                 tblock = uint8(tframe(y+i+yoffset:y+i+yoffset+bsize-1,...
%                     x+j+xoffset:x+j+xoffset+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
               % if a block with a lower MAD is found, keep track of the
               % adjustments, and save new MAD value
                if (MAD < minMAD)
%                     subplot(1,3,3),imshow(tblock);   % for debug                 
%                     title('best block'); % for debug
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
%                 tblock = uint8(tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
%                     x+j:x+j+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
                if (MAD < minMAD)
%                     subplot(1,3,3),imshow(tblock);   % for debug                 
%                     title('best block'); % for debug
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
%                 tblock = uint8(tframe(y+i:y+i+bsize-1,...
%                     x+2*xoffset:x+2*xoffset+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
                if (MAD < minMAD)
%                     subplot(1,3,3),imshow(tblock);   % for debug                 
%                     title('best block'); % for debug
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
%             tblock = uint8(tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
%                 x+2*xoffset:x+2*xoffset+bsize-1));  % for debug 
%             subplot(1,3,2),imshow(tblock); % for debug
%             title('current block'); % for debug
           % if a block with a lower MAD is found, keep track of the
           % adjustments, and save new MAD value
            if (MAD < minMAD)
%                 subplot(1,3,3),imshow(tblock);   % for debug                 
%                 title('best block'); % for debug
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
%                 tblock = uint8(tframe(y+2*yoffset:y+2*yoffset+bsize-1,...
%                     x+j+2*xoffset:x+j+2*xoffset+bsize-1));  % for debug 
%                 subplot(1,3,2),imshow(tblock); % for debug
%                 title('current block'); % for debug
               % if a block with a lower MAD is found, keep track of the
               % adjustments, and save new MAD value
                if (MAD < minMAD)
%                     subplot(1,3,3),imshow(tblock);   % for debug                 
%                     title('best block'); % for debug
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
%             tblock = uint8(tframe(y+i+2*yoffset:y+i+2*yoffset+bsize-1,...
%                 x+2*xoffset:x+2*xoffset+bsize-1));  % for debug 
%             subplot(1,3,2),imshow(tblock); % for debug
%             title('current block'); % for debug
              % if a block with a lower MAD is found, keep track of the
              % adjustments, and save new MAD value
                if (MAD < minMAD)
%                     subplot(1,3,3),imshow(tblock);   % for debug                 
%                     title('best block'); % for debug
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
    mvframe((y-1)/bsize+1,(x-1)/bsize+1,1) = xoffset;
    mvframe((y-1)/bsize+1,(x-1)/bsize+1,2) = yoffset;
    % predict the anchor frame from the target frame and motion vector 
   pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+yoffset:y+yoffset+bsize-1,...
        x+xoffset:x+xoffset+bsize-1);
end
end
figure;
% removes the zero padding from macroblocking
pframe = pframe(1:end-hremain,1:end-wremain);
aframe = aframe(1:end-hremain,1:end-wremain);
% plots the motion vectors for each block
quiver(mvframe(:,:,1),mvframe(:,:,2));
title(sprintf('New-3-Step Motion Vector Field: BlockSize = %d',bsize));
psnr = 10*log10(255*255/immse(pframe,aframe)); 
eframe = pframe - aframe; % residual frame between actual and predicted 
pframe = uint8(pframe);
eframe = uint8(abs(eframe));
figure;
imshow(eframe),
title(sprintf('New-3-Step Residual Image: BlockSize = %d',bsize));
figure;
imshow(pframe),
title(sprintf('New-3-Step Predicted Frame: BlockSize = %d,PSNR = %0.2f',...
    bsize,psnr));
end