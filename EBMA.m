function [pframes] = EBMA(aframes,tframes,bsize,rwidth)
% exhaustive block matching algorithm forward motion compensation:
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

aframe = double(aframes(:,:,1));
pframes = zeros(size(aframes));
tframe = double(tframes(:,:,1));
[height, width] = size(aframe);
wremain = mod(width,bsize);
hremain = mod(height,bsize);
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
Rpframe = zeros(modifiedsize(1),modifiedsize(2));
Gpframe = zeros(modifiedsize(1),modifiedsize(2));
Bpframe = zeros(modifiedsize(1),modifiedsize(2));
Rtframe = zeros(modifiedsize(1),modifiedsize(2));
Gtframe = zeros(modifiedsize(1),modifiedsize(2));
Btframe = zeros(modifiedsize(1),modifiedsize(2));
Rtframe(1:height,1:width) = tframes(:,:,1);
Gtframe(1:height,1:width) = tframes(:,:,2);
Btframe(1:height,1:width) = tframes(:,:,3);
for y = 1:bsize:height
    for x = 1:bsize:width
        MAD = 255; %max error possible
        minMAD = MAD; 
        dx = 0;
        dy = 0;
        % for valid pixels in the region compare blocks using MeanAbsDiff
        for i = -rwidth:rwidth
            for j = -rwidth:rwidth
                if(y+i>0 && y+i+bsize-1<height+1 && ...
                        x+j>0 && x+j+bsize-1<width+1)
                    MAD = sum(sum(abs(aframe(y:y+bsize-1,x:x+bsize-1) ...
                          - tframe(y+i:y+i+bsize-1,x+j:x+j+bsize-1)))) ...
                          /(bsize*bsize);
                end
               % if a block with lower MAD is found keep track of the
               % displacment vector, and save new MAD value
                if MAD < minMAD             
                    minMAD = MAD;
                    dx = j;
                    dy = i;
                end
            end
        end
        % predict the anchor frame from the target frame and motion vector 
        Rpframe(y:y+bsize-1,x:x+bsize-1) = Rtframe(y+yoffset:...
            y+yoffset+bsize-1,x+xoffset:x+xoffset+bsize-1);
        Gpframe(y:y+bsize-1,x:x+bsize-1) = Gtframe(y+yoffset:...
            y+yoffset+bsize-1,x+xoffset:x+xoffset+bsize-1);
        Bpframe(y:y+bsize-1,x:x+bsize-1) = Btframe(y+yoffset:...
            y+yoffset+bsize-1,x+xoffset:x+xoffset+bsize-1);
    end
end
% removes the zero padding from macroblocking
pframes(:,:,1) = Rpframe(1:end-hpads,1:end-wpads);
pframes(:,:,2) = Gpframe(1:end-hpads,1:end-wpads);
pframes(:,:,3) = Bpframe(1:end-hpads,1:end-wpads);
end