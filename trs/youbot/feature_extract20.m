%HOUGH Summary of this function goes here
%   Detailed explanation goes here
close all;
I = imread('sim.png');

% w=fspecial('gaussian',[5 5],3);
% I=imfilter(I,w);
%I = imadjust(I);
BW_B=im2bw(I);
BW = edge(BW_B,'sobel');
figure,imshow(BW);
%hough变换
[H,theta,rho] = hough(BW);
%使用houghpeaks寻找峰值
P = houghpeaks(H,20,'threshold',ceil(0.1*max(H(:))));
size(H)
size(theta)
size(rho)
P
lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',20);
%
%在原始的图像上叠加直线
figure,imshow(I),hold on
max_len = 0;
for k=1:length(lines)
    xy = [lines(k).point1;lines(k).point2];
    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','yellow');
    len = norm(lines(k).point1 - lines(k).point2);
end

param = findcircle(BW,1,0.02,60,150,0.55);% circle on the ball
%param = findcircle(BW,1,0.01,60,200,0.95);% circle on the hat
%param = findcircle(BW,1,0.1,60,200,0.85);

for i =1:size(param,1)
    x0 = param(i,1); y0 = param(i,2); r0 = param(i,3);
    xi = [-r0:0,0:r0];
    yi = round((r0^2 - xi.^2).^0.5);
    plot(yi+y0,xi+x0,'Color','g','LineWidth',3);
    plot(-yi+y0,xi+x0,'Color','g','LineWidth',3);
end

clear


%
% end