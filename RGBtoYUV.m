function [ YUV ] = RGBtoYUV( RGB )
%  Converts RGB color coordinates into YUV coordinates
%   input arg: RGB a NxNx3 rgb matrix of type double
R = RGB(:,:,1);
G = RGB(:,:,2);
B = RGB(:,:,3);
Y = 0.299*R + 0.587*G + 0.114*B;
U = -0.147*R - 0.289*G + 0.436*B;
V = 0.615*R - 0.515*G - 0.1*B;
YUV = cat(3,Y,U,V);
% YUV = [0.299 0.587 0.114;-0.147 -0.289 0.436;0.615 -0.515 -0.1]*RGB;

end

