function i = ij(x)
%��������ת��Ϊ��ɢ����
  global cellsize;
  n = 15/cellsize;
  i = max(min(floor(x/cellsize)+n/2+1, n), 1);
end