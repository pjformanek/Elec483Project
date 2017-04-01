bsize = 16;
pframes = zeros(872,2048,3);
framecount = 0;
vidReadObj = VideoReader('sintel-2048-stereo.mp4');
vidReadObj.CurrentTime = 163;
vidWriteObj = VideoWriter(sprintf('sintel_NTS%d.mp4',bsize),'MPEG-4');
vidWriteObj.FrameRate = vidReadObj.FrameRate;
open(vidWriteObj);
vidWriteObj2 = VideoWriter(sprintf('sintel_NTS%d_orig.mp4',bsize),'MPEG-4');
vidWriteObj2.FrameRate = vidReadObj.FrameRate;
open(vidWriteObj2);
aframe = readFrame(vidReadObj);
for i = 1:240
    tframe = aframe;
    aframe = readFrame(vidReadObj);
    writeVideo(vidWriteObj2, aframe);
    for j = 1:3
    [pframe, mvframe] = NewThreeStep(aframe(:,:,j),tframe(:,:,j),bsize);
    pframes(:,:,j) = pframe;
    end
    pframes = uint8(pframes);
    writeVideo(vidWriteObj, pframes);
end
close(vidWriteObj);
close(vidWriteObj2);
   