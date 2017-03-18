function [pframe, MVframe] = EBMA(aframe,tframe,bsize,rwidth)
% exhaustive block matching algorithm forward motion compensation:
%  @arg
%       aframe: anchor/current frame
%       tframe: target/reference frame
%       bsize: block size
%       rwidth:  region width 2*rwidth
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
        dx = 0;
        dy = 0;
%         ablock = uint8(aframe(y:y+bsize-1,x:x+bsize-1)); % used for debug
%         subplot(1,2,1),imshow(ablock);  % used for debug
       % for valid pixels in the region compare blocks using MeanAbsDiff
        for i = -rwidth:rwidth
            for j = -rwidth:rwidth
                if(y+i>0 && y+i+bsize-1<height+1 && x+j>0 && x+j+bsize-1<width+1)
                    MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                          - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1))))/(bsize*bsize);
% used for debug    tblock = uint8(tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1));                 
                end
               % if a block with lower MAD is found keep track of the
               % displacment vector, and save new MAD value
                if MAD < minMAD
% used for debug    subplot(1,2,2),imshow(tblock);                    
                    minMAD = MAD;
                    dx = j;
                    dy = i;
                end
            end
        end
        % build up the motion vectors of the frame
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,1) = dy;
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,2) = dx;
        % predict the anchor frame from the target frame and motion vector 
        pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+dy:y+dy+bsize-1,x+dx:x+dx+bsize-1);
    end
end
figure;
% plots the motion vectors for each block
quiver(MVframe(:,:,1),MVframe(:,:,2));
title(sprintf('Motion Vector Field using search region = %d',rwidth));
psnr = 10*log10(255*255/immse(pframe,aframe)); 
eframe = pframe - aframe; % residual frame between actual and predicted 
pframe = uint8(pframe);
eframe = uint8(eframe);
figure;
imshow(eframe),title(sprintf('Residual Image using search region = %d',rwidth));
figure;
imshow(pframe),
title(sprintf('Predicted Frame: R = %d, PSNR = %0.2f',rwidth,psnr));
end