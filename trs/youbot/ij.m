function i = ij(x)
  cellsize=0.25;
  n = 15/cellsize;
  i = max(min(floor(x/cellsize)+n/2+1, n), 1);
end