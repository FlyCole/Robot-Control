function x = xy(i)
%%离散坐标转化为连续坐标
  global cellsize;
  n = 15/cellsize;
  x = (i-n/2-1)*cellsize+cellsize/2;
end