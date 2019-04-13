function handles=PX3_init(vrep,id)

handles = struct('id', id);



[res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_left', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_right', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%%%%%%%获取PX3的关节
[res robot_body] = vrep.simxGetObjectHandle(id, 'robot_body', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.wheelJoints = wheelJoints;
handles.robot_body=robot_body;


res = vrep.simxGetObjectPosition(id, robot_body, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, robot_body, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);






end