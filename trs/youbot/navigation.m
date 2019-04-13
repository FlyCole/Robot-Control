goal=[ij(-6.8),ij(-2.25)]; %设置目标坐标（使用离散地图坐标系）
    dx=DXform(double(fmap),'private'); %生成距离地图
    dx.plan([goal(1);goal(2)]); %按目标计算距离
    traj=dx.path([ij(robotPos(1));ij(robotPos(2))]); %规划最优路径（离散地图下的坐标序列）
    traj=[xy(traj(1,:));xy(traj(2,:))]; %转换为连续地图路径
    traj=[traj [-6.8;-2.25]]; %补充最后一个点
    target = traj(:,1) %目标点永远为第一个点
    traj = traj(:,2:end); %轨迹永远从第二个点开始算起
