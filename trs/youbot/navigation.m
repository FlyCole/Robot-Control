goal=[ij(-6.8),ij(-2.25)]; %����Ŀ�����꣨ʹ����ɢ��ͼ����ϵ��
    dx=DXform(double(fmap),'private'); %���ɾ����ͼ
    dx.plan([goal(1);goal(2)]); %��Ŀ��������
    traj=dx.path([ij(robotPos(1));ij(robotPos(2))]); %�滮����·������ɢ��ͼ�µ��������У�
    traj=[xy(traj(1,:));xy(traj(2,:))]; %ת��Ϊ������ͼ·��
    traj=[traj [-6.8;-2.25]]; %�������һ����
    target = traj(:,1) %Ŀ�����ԶΪ��һ����
    traj = traj(:,2:end); %�켣��Զ�ӵڶ����㿪ʼ����
