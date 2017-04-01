bsize = 16;
pframes = zeros(360,640,3);
writerObj = VideoWriter('TS16prediction2.avi');
writerObj.FrameRate = 24;
open(writerObj);
for i = 1:23
    aframe = sprintf('BBB00%d.png',i+149);
    tframe = sprintf('BBB00%d.png',i+148);
    aframe = imread(aframe);
    tframe = imread(tframe);
    for j = 1:3
    [pframe, mvframe] = ThreeStep(aframe(:,:,j),tframe(:,:,j),bsize,7);
    pframes(:,:,j) = pframe;
    end
    pframes = uint8(pframes);
%     imwrite(pframes,sprintf('pTS_BBB0%d.png',i+4198));
    writeVideo(writerObj, pframes);
end
close(writerObj);
