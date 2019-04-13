function i = ij(x)
%连续坐标转化为离散坐标
  global cellsize;
  n = 15/cellsize;
  i = max(min(floor(x/cellsize)+n/2+1, n), 1);
end