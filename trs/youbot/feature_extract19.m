%HOUGH Summary of this function goes here
%   Detailed explanation goes here
close all;
I = imread('sim.png');

%% yellow feature extract
% a = I;
% modeblue=(a(:,:,1)>150).*(a(:,:,2)>150).*(a(:,:,3)<100);
% a(:,:,1)=255*modeblue;
% a(:,:,2)=255*modeblue;
% a(:,:,3)=255*modeblue;
% figure,imshow(uint8(a));

diff_R = 100;
Image=I;
RP_R=Image(:,:,1); 
RP_G=Image(:,:,2); 
RP_B=Image(:,:,3);
XYR=255*((RP_R-RP_G)>diff_R&(RP_R-RP_B)>diff_R); 
figure,imshow(uint8(XYR));

%b=rgb2gray(a);
bw=im2bw(XYR);
[r c]=find(bw==1);   
[rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
line(rectx(:),recty(:),'color','r');

rectx
recty
%
% end