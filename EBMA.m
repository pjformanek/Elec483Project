function [pframe, MVframe] = EBMA(aframe,tframe,bsize,rwidth)
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
        % build up the motion vectors of the frame
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,1) = dy;
        MVframe((y-1)/bsize+1,(x-1)/bsize+1,2) = dx;
        % predict the anchor frame from the target frame and motion vector 
        pframe(y:y+bsize-1,x:x+bsize-1) = tframe(y+dy:y+dy+bsize-1,...
            x+dx:x+dx+bsize-1);
    end
end
% removes the zero padding from macroblocking
pframe = pframe(1:end-hpads,1:end-wpads);
end