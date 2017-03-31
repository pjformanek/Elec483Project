function pframes = ChromaPredict( pframe, tframes, mvframe, bsize)
[height,width,frame] = size(tframes);
pframes = zeros(height,width,frame);
pframes(:,:,1) = pframe;
wremain = mod(width,bsize);
hremain = mod(height,bsize);
% adds zero padding for integer division of frame into macroblocks
if((wremain ~= 0) || (hremain ~=0))
   pframes = padarray(pframes,[hremain,wremain,0],'post');
   tframes = padarray(tframes,[hremain,wremain,0],'post');
end

for chroma = 2:3
    for row = 1:bsize:height
        for col = 1:bsize:width
            pframes(col:col+bsize-1,row:row+bsize-1,chroma) = ...
                tframes(col+mvframe(ceil(row/bsize)+1,ceil(col/bsize)+1,2)...
                :col+mvframe(ceil(row/bsize)+1,ceil(col/bsize)+1,2)+bsize-1,...
                row+mvframe(ceil(row/bsize)+1,ceil(col/bsize)+1,1):...
                row+mvframe(ceil(row/bsize)+1,ceil(col/bsize)+1,1)+bsize-1,chroma);
        end
    end
end