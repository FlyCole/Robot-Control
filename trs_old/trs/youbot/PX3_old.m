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
    %С�����ӵİ뾶
    r = 0.0975;
    %С���־�뾶
    b = 0.1655;
    %С���ٶ�
    forwBackVel = 0;
    rotVel = 0;
    %P����
    Kv=0.1;
    Kh=5;
    pause(1);
    
    % One map cell is 25cm large
    global cellsize;
    cellsize = .25;

    pause(.3);
    t = 0;
    res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
    
    
    fsm='rotate';
    while true,
        [res,robotPos] = vrep.simxGetObjectPosition(h.id, h.robot_body, -1,vrep.simx_opmode_buffer); 
        robotPos
        vrchk(vrep, res);
        [res,robotEuler] = vrep.simxGetObjectOrientation(h.id, h.robot_body, -1,vrep.simx_opmode_buffer); 
        vrchk(vrep, res);

        %�����յ�����target
        target(1)=3;%xy(goal(2))
        target(2)=-2;%xy(goal(1))
        
        if strcmp(fsm, 'rotate'),
            P_ow=[target(1);target(2);1];%Ŀ�����������
            T_cw=se2(robotPos(1),robotPos(2),robotEuler(3))%����robot����ϵ����������ϵ�е�ת�ƾ���
            P_oc=inv(T_cw)*P_ow;  %��ȡ��robot����ϵ�µ�Ŀ�������
            %%�ж�����
            if P_oc(1)<0 && P_oc(2)<0 %��������
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle-pi;   %atan�����Ϊ��һ���޽�
            end
            if P_oc(1)<0 && P_oc(2)>0 %�ڶ�����
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle+pi;   %atan�����Ϊ�������޽�
            end
            if P_oc(1)>0 %��һ��������
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle;
            end
            if P_oc(1)==0 && P_oc(2)>0 %y���ϰ���
                angl=pi/2;
            end
            if P_oc(1)==0 && P_oc(2)<0 %y���°���
                angl=-pi/2;
            end
            if P_oc(2)==0 
                angl=0;
            end
            rotVel =Kh*angl;%����rotVel
            fsm = 'drive';%״̬����Ϊ'drive'
            
        elseif strcmp(fsm, 'drive'),
            forwBackVel =Kv*sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2);%�����ٶ�forwBackVel
            fsm = 'rotate';   
            if sqrt((robotPos(1)-target(1))^2+(robotPos(2)-target(2))^2)<0.01 %�ﵽ�ɿؼ���
                fsm='finished';%״̬����Ϊ'finished'
            end
            
        elseif strcmp(fsm, 'finished'),
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
          plot( robotPos(1), robotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');%������ǰ���Ŀ���ͼ��
          axis equal;
          axis([-7.8 7.8 -7.8 7.8]);
        end
        drawnow;
        vrep.simxSynchronousTrigger(id);
        t = t+timestep;
    end
    disp('Program ended');
end

