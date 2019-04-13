function [para] = findcircle(BW,stepR,stepAngle,minR,maxR,p)

[m,n] = size(BW);% BW
cntR = round((maxR-minR)/stepR)+1;% stepR: minR: maxR:
cntAngle = round(2*pi/stepAngle);% stepAngle:
hough_space = zeros(m,n,cntR);% hough_space: h(a,b,r) (a,b) r 
[rows,cols] = find(BW);
cntPoints = size(rows,1);
 
% Hough�任��ͼ��ռ�(x,y)��Ӧ�������ռ�(a,b,r)
% a = x-r*cos(angle), b = y-r*sin(angle)
for i=1:cntPoints
    for r=1:cntR
        for k=1:cntAngle
            a = round(rows(i)-(minR+(r-1)*stepR)*cos(k*stepAngle));
            b = round(cols(i)-(minR+(r-1)*stepR)*sin(k*stepAngle));
            if(a>0 && a<=m && b>0 && b<=n)
                hough_space(a,b,r) = hough_space(a,b,r)+1;
            end
        end
    end
end
 
% Ѱ��������ֵ��Բ�Ĳ���
max_para = max(max(max(hough_space)));
index = find(hough_space>=max_para*p); % p:��p*hough_space�����ֵΪ��ֵ��pȡ0��1֮�����
length = size(index,1);
hough_circle=zeros(m,n);
for i=1:cntPoints
    for k=1:length
        par3 = floor(index(k)/(m*n))+1;
        par2 = floor((index(k)-(par3-1)*(m*n))/m)+1;
        par1 = index(k)-(par3-1)*(m*n)-(par2-1)*m;
        if((rows(i)-par1)^2+(cols(i)-par2)^2<(minR+(par3-1)*stepR)^2+5 && (rows(i)-par1)^2+(cols(i)-par2)^2>(minR+(par3-1)*stepR)^2-5)
            hough_circle(rows(i),cols(i)) = 1;% hough_circl:��ֵͼ�񣬼�⵽��Բ
        end
    end
end
 
for k=1:length
    par3 = floor(index(k)/(m*n))+1;     
    par2 = floor((index(k)-(par3-1)*(m*n))/m)+1;    % Բ��y����
    par1 = index(k)-(par3-1)*(m*n)-(par2-1)*m;      % Բ��x����
    par3 = minR+(par3-1)*stepR;                    % Բ�İ뾶
   % fprintf(1,'Center %d %d radius %d\n',par1,par2,par3);
    para(k,:) = [par1,par2,par3];  % para:��⵽��Բ��Բ�ġ��뾶
end
