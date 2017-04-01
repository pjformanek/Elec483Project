bsize = 16;
aframes = imread('big_buck_bunny_04210_low.png');
tframes = imread('big_buck_bunny_04209_low.png');
% [pframes , mvframes] = ChromaPredict(aframes,tframes,16);
figure;
image(aframes);
title('Anchor frame');
figure;
image(tframes);
title('Target frame');
[height, width, YUV]=size(aframes);
EBMApframes = zeros(height, width, YUV);
EBMAmvframes = zeros(ceil(height/bsize),ceil(width/bsize),2,YUV);
TSpframes = zeros(height, width, YUV);
TSmvframes = zeros(ceil(height/bsize),ceil(width/bsize),2,YUV);
NTSpframes = zeros(height, width, YUV);
NTSmvframes = zeros(ceil(height/bsize),ceil(width/bsize),2,YUV);
for i = 1:YUV
    [pframe ,mvframe] = NewThreeStep(aframes(:,:,i),tframes(:,:,i),bsize);
    NTSpframes(:,:,i) = pframe;
    NTSmvframes(:,:,1,i) = mvframe(:,:,1);
    NTSmvframes(:,:,2,i) = mvframe(:,:,2);
end 
figure;
image(uint8(NTSpframes));
title('New-Three-Step');
for i = 1:YUV
    [pframe ,mvframe] = ThreeStep(aframes(:,:,i),tframes(:,:,i),bsize,7);
    TSpframes(:,:,i) = pframe;
    TSmvframes(:,:,1,i) = mvframe(:,:,1);
    TSmvframes(:,:,2,i) = mvframe(:,:,2);
end
figure;
image(uint8(TSpframes));
title('Three-Step');
