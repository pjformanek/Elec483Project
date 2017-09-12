% used to put videos together 
demovid1 = VideoWriter('demo4.mp4','MPEG-4');
vidObj = VideoReader('Ashton.mp4');
demovid1.FrameRate = vidObj.FrameRate;
open(demovid1);
while hasFrame(vidObj)
    writeVideo(demovid1,readFrame(vidObj));
end
vidObj = VideoReader('Ash_NTS16.mp4');
while hasFrame(vidObj)
    writeVideo(demovid1,readFrame(vidObj));
end
close(demovid1);