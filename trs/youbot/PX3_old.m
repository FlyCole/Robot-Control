function PX3()

    disp('Program started');
    %Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connexion whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service"
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
    % We're not checking the error code - if vrep is not run in continuous remote
    % mode, simxStartSimulation could return an error.
    % vrchk(vrep, res);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose,
    % the Hokuyo, and the arm tip pose.
    h = PX3_init_old(vrep, id);
    timestep = .05;
    %小车轮子的半径
    r = 0.0975;
    %小车轮距半径
    b = 0.1655;
    %小车速度
    forwBackVel = 0;
    rotVel = 0;
    %P控制
    Kv=0.5;
    Kh=0.5;
    pause(1);
    
    % One map cell is 25cm large
    global cellsize;
    cellsize = .25;

    pause(.3);
    t = 0;
    res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
    
    %map_building
    [res,robotPos] = vrep.simxGetObjectPosition(h.id, h.robot_body, -1,vrep.simx_opmode_buffer); 
    ij(robotPos(1))
    vrchk(vrep, res);%获取初始位置 init_position capture
    goal=[ij(6),ij(0)]; %设置目标坐标（使用离散地图坐标系） set goal position(using discrete map system)
    load('fmap.mat');
    %fmap=double(fmap);
    %fmap=rot90(fmap,2);
    dx=DXform(double(fmap),'private'); %生成距离地图 generate distance map
    dx.plan([goal(1);goal(2)]); %按目标计算距离 distance-calculating
    traj=dx.path([ij(robotPos(1));ij(robotPos(2))])'; %规划最优路径（离散地图下的坐标序列）plan the best route(discrete)
    traj=[xy(traj(1,:));xy(traj(2,:))]; %转换为连续地图路径 change to continuous
    traj=[traj [6;0]]; %补充最后一个点 add the last point
    %traj=traj(:,2:end);
    
    for k=1:1:size(traj,2)
         plot(traj(1,k),traj(2,k),'*b');
         hold on;
    end
     
    fsm='rotate';
    while true,
        [res,robotPos] = vrep.simxGetObjectPosition(h.id, h.robot_body, -1,vrep.simx_opmode_buffer); 
        vrchk(vrep, res);
        [res,robotEuler] = vrep.simxGetObjectOrientation(h.id, h.robot_body, -1,vrep.simx_opmode_buffer); 
        vrchk(vrep, res);
        
        %给出当前终点向量target
        traj
        target = traj(:,1); %目标点永远为第一个点

        %target(1)=3;%xy(goal(2))
        %target(2)=-2;%xy(goal(1))
        
        
        
        if strcmp(fsm, 'rotate'),
            P_ow=[target(1);target(2);1];%目标点世界坐标
            T_cw=se2(robotPos(1),robotPos(2),robotEuler(3));%建立robot坐标系在世界坐标系中的转移矩阵
            P_oc=inv(T_cw)*P_ow;  %获取在robot坐标系下的目标点坐标
            %%判断象限
            if P_oc(1)<0 && P_oc(2)<0 %第三象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle-pi;   %atan计算出为第一象限角
            end
            if P_oc(1)<0 && P_oc(2)>0 %第二象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle+pi;   %atan计算出为第四象限角
            end
            if P_oc(1)>0 %第一、二象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle;
            end
            if P_oc(1)==0 && P_oc(2)>0 %y轴上半轴
                angl=pi/2;
            end
            if P_oc(1)==0 && P_oc(2)<0 %y轴下半轴
                angl=-pi/2;
            end
            if P_oc(2)==0 
                angl=0;
            end
            rotVel =Kh*angl;%更新rotVel
            fsm = 'drive';%状态更改为'drive'
            
        elseif strcmp(fsm, 'rotate2'),%最后一点（目标点）
            P_ow=[target(1);target(2);1];%目标点世界坐标
            T_cw=se2(robotPos(1),robotPos(2),robotEuler(3));%建立robot坐标系在世界坐标系中的转移矩阵
            P_oc=inv(T_cw)*P_ow;  %获取在robot坐标系下的目标点坐标
            %%判断象限
            if P_oc(1)<0 && P_oc(2)<0 %第三象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle-pi;   %atan计算出为第一象限角
            end
            if P_oc(1)<0 && P_oc(2)>0 %第二象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle+pi;   %atan计算出为第四象限角
            end
            if P_oc(1)>0 %第一、二象限
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle;
            end
            if P_oc(1)==0 && P_oc(2)>0 %y轴上半轴
                angl=pi/2;
            end
            if P_oc(1)==0 && P_oc(2)<0 %y轴下半轴
                angl=-pi/2;
            end
            if P_oc(2)==0 
                angl=0;
            end
            rotVel =Kh*angl;%更新rotVel
            fsm = 'drive2';%状态更改为'drive'
            
        elseif strcmp(fsm, 'drive'),
            forwBackVel =Kv*sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2);%更新速度forwBackVel
            fsm = 'rotate';   
            if sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2)<1 %达到可控极限
                fsm='finished';%状态更改为'finished'
            end
            
        elseif strcmp(fsm, 'drive2'),%最后一点（目标点）
            forwBackVel =Kv*sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2);%更新速度forwBackVel
            fsm = 'rotate2';   
            if sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2)<0.05 %达到可控极限
                fsm='end';%状态更改为'finished'
            end
            
        elseif strcmp(fsm, 'finished'),
            [~,y]=size(traj);
            if y==1
                fsm='rotate2'
                continue;
            end
            traj = traj(:,2:end); %轨迹永远从第二个点开始算起
            fsm='rotate';
            continue
            
        elseif strcmp(fsm,'end'),%到达目标点
            break;
        end

        % Update wheel velocities
        res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
        vLeft = -(forwBackVel - b * rotVel) / r;
        vRight = -(forwBackVel + b * rotVel) / r;
        vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
        vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
        res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);

        if 1,
          [~,~] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),...
                         (-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
          plot( robotPos(1), robotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');%做出当前点和目标点图像
          axis equal;
          axis([-7.8 7.8 -7.8 7.8]);
        end
        drawnow;
        vrep.simxSynchronousTrigger(id);
        t = t+timestep;
    end
    disp('Program ended');
end

