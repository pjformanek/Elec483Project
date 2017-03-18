function [pframe, MVframe] = ThreeStep(aframe,tframe,bsize,R)
% 3Step search block matching algorithm for motion compensation:
%  @arg
%       aframe: anchor/current frame
%       tframe: target/reference frame
%       bsize: block size
%       R:  search range, the total search area is ((2*R+1)^2
%  @output
%       pframe: Predicted frame
%       MVframe:  Motion vectors, using multidimensional array
%              MVframe(:,:,1) are all the dx values
%              MVframe(:,:,2) are all the dy values

[height, width] = size(aframe);
aframe = double(aframe);
tframe = double(tframe);
MVframe = zeros(height/bsize,width/bsize,2);
pframe = zeros(height,width);
for y = 1:bsize:height
    for x = 1:bsize:width
        MAD = 255; %max error possible
        minMAD = MAD; 
        % reset the motion vectors and offsets for each block
        range = R;
        mvx = 0;
        mvy = 0;
        xoffset = 0;
        yoffset = 0;
%         ablock = uint8(aframe(y:y+bsize-1,x:x+bsize-1)); % used for debug
%         subplot(1,3,1),imshow(ablock);  % used for debug
%         title('target block'); % for debug

% searches till the range is 1 and compares using MeanAbsDiff
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
%         tblock = uint8(tframe(y:y+bsize-1,x:x+bsize-1));  % for debug
%         subplot(1,3,2),imshow(tblock);   % for debug 
%         title('current block'); % for debug
%         subplot(1,3,3),imshow(tblock);   % for debug                 
%         title('best block');    % for debug
        minMAD = MAD;
    end
    for i = -range:range:range
        for j = -range:range:range
            % if the current search block is in the image bounds
            if(y+i+yoffset>0 && y+i+yoffset+bsize-1<height+1 &&...
                    x+j+xoffset>0 && x+j+xoffset+bsize-1<width+1 && ...
                    (i > 0 || j> 0))
               % compare blocks between images
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
%                 subplot(1,3,3),imshow(tblock);   % for debug                 
%                 title('best block'); % for debug
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
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,1) = mvx;
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,2) = mvy;
        % predict the anchor frame from the target frame and motion vector 
        pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+mvy:y+mvy+bsize-1,...
            x+mvx:x+mvx+bsize-1);
    end
end
figure;
% plots the motion vectors for each block
quiver(MVframe(:,:,1),MVframe(:,:,2));
title(sprintf('Motion Vector Field using R = %d',R));
psnr = 10*log10(255*255/immse(pframe,aframe)); 
eframe = pframe - aframe; % residual frame between actual and predicted 
pframe = uint8(pframe);
eframe = uint8(eframe);
figure;
imshow(eframe),title(sprintf('Residual Image using R = %d',R));
figure;
imshow(pframe),
title(sprintf('Predicted Frame: R = %d, PSNR = %0.2f',R,psnr));
end