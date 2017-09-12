% used to build a video made of predicted frames from the motion vectors
bsize = 16;
vidReadObj = VideoReader('Ashton.mp4');
% vidReadObj.CurrentTime = 420; 
pframes = zeros(vidReadObj.Height,vidReadObj.Width,3);
vidWriteObj = VideoWriter(sprintf('Ash_NTS%d.mp4',bsize),'MPEG-4');
vidWriteObj.FrameRate = vidReadObj.FrameRate;
open(vidWriteObj);
% vidWriteObj2 = VideoWriter('BBB_480p_orig.avi');
% vidWriteObj2.FrameRate = vidReadObj.FrameRate;
% open(vidWriteObj2);
aframe = readFrame(vidReadObj);
% for i = 1:360
while hasFrame(vidReadObj)
    tframe = aframe;
    aframe = readFrame(vidReadObj);
%     writeVideo(vidWriteObj2, aframe);
    pframes = NewThreeStep(aframe,tframe,bsize);
    pframes = uint8(pframes);
    writeVideo(vidWriteObj, pframes);
end
close(vidWriteObj);
% close(vidWriteObj2);