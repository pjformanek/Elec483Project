function [pframe, MVframe] = EBMAdemo(aframe,tframe,bsize,rwidth)
% exhaustive block matching algorithm forward motion compensation:
% displays the blocks during the the block matching 
% set the initial x and y values in the for loops to start the demo at 
% a specific point in the image, start must be a multiple of the block size
% plus one. (eg. x = 5*bsize+1:bsize:height)
%  @arg
%       aframe: anchor/current frame
%       tframe: target/reference frame
%       bsize: block size
%       rwidth: total search region width = 2*rwidth
%  @output
%       pframe: Predicted frame
%       MVframe:  Motion vectors, using multidimensional array
%              MVframe(:,:,1) are all the dx values
%              MVframe(:,:,2) are all the dy values

figure;
subplot(1,2,1);
imshow(aframe);
title('Anchor frame');
subplot(1,2,2);
imshow(tframe);
title('Target frame');
figure;
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
for y = 14*bsize+1:bsize:height
    for x = 10*bsize+1:bsize:width
        MAD = 255; %max error possible
        minMAD = MAD; 
        dx = 0;
        dy = 0;
        ablock = uint8(aframe(y:y+bsize-1,x:x+bsize-1)); % for debug
        subplot(1,3,1),imshow(ablock);  % for debug
        title('anchor block'); % for debug
%        for valid pixels in the region compare blocks using MeanAbsDiff
        for i = -rwidth:rwidth
            for j = -rwidth:rwidth
                if(y+i>0 && y+i+bsize-1<height+1 && x+j>0 && x+j+bsize-1<width+1)
                    MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                          - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1))))/(bsize*bsize);
                    tblock = uint8(tframe(y+i:y+i+bsize-1,x+j: ...
                        x+j+bsize-1)); % for debug        
                    subplot(1,3,2),imshow(tblock); % for debug
                    title('target block'); % for debug
                end
               % if a block with lower MAD is found keep track of the
               % displacment vector, and save new MAD value
                if MAD < minMAD
                    subplot(1,3,3),imshow(tblock); %  for debug
                    title(sprintf('best MAD:%.2f',MAD)); % for debug
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
% removes the zero padding from macroblocking
pframe = pframe(1:end-hpads,1:end-wpads);
plots the motion vectors for each block
figure;
quiver(MVframe(:,:,1),MVframe(:,:,2));
title(sprintf('Motion Vector Field: BlockSize = %d, R = %d',bsize,rwidth));
psnr = 10*log10(255*255/immse(pframe,aframe)); 
eframe = pframe - aframe; % residual frame between actual and predicted 
pframe = uint8(pframe);
eframe = uint8(abs(eframe));
figure;
imshow(eframe),
title(sprintf('Residual Image: BlockSize = %d, R = %d',bsize,rwidth));
figure;
imshow(pframe),
title(sprintf('Predicted Frame: BlockSize = %d, R = %d, PSNR = %0.2f',...
    bsize,rwidth,psnr));
end