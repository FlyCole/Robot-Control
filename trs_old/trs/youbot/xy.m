function x = xy(i)
%%��ɢ����ת��Ϊ��������
  global cellsize;
  n = 15/cellsize;
  x = (i-n/2-1)*cellsize+cellsize/2;
end