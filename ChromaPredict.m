function [pframes, mvframes] = ChromaPredict( aframes, tframes, bsize)

[height, width, YUV]=size(aframes);
pframes = zeros(height, width, YUV);
mvframes = zeros(ceil(height/bsize),ceil(width/bsize),2,YUV);
for i = 1:YUV
    [pframe ,mvframe] = NewThreeStep(aframes(:,:,i),tframes(:,:,i),bsize);
    pframes(:,:,i) = pframe;
    mvframes(:,:,1,i) = mvframe(:,:,1);
    mvframes(:,:,2,i) = mvframe(:,:,2);
end
end