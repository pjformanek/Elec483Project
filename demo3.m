bsize = 16;
pframes = zeros(872,2048,3);
framecount = 0;
vidReadObj = VideoReader('sintel-2048-stereo.mp4');
vidReadObj.CurrentTime = 60;
fname = sprintf('sintel_NTS%d.mp4',bsize);
vidWriteObj = VideoWriter(fname,'MPEG-4');
vidWriteObj.FrameRate = vidReadObj.FrameRate;
open(vidWriteObj);
aframe = readFrame(vidReadObj);
for i = 1:24
    tframe = aframe;
    aframe = readFrame(vidReadObj);
    for j = 1:3
    [pframe, mvframe] = NewThreeStep(aframe(:,:,j),tframe(:,:,j),bsize);
    pframes(:,:,j) = pframe;
    end
    pframes = uint8(pframes);
    writeVideo(vidWriteObj, pframes);
end
close(vidWriteObj);
   