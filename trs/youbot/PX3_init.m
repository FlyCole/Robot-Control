function handles=PX3_init(vrep,id)

handles = struct('id', id);



[res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'left_wheel_Revolute_joint', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'right_wheel_Revolute_joint', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%%%%%%%获取PX3的关节
[res Pioneer_p3dx] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_body_visible', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.wheelJoints = wheelJoints;
handles.Pioneer_p3dx=Pioneer_p3dx;

% [res wheelJoints(1)] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_leftMotor#0', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% [res wheelJoints(2)] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_rightMotor#0', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
% %%%%%%%获取PX3的关节
% [res Pioneer_p3dx] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx#0', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

res = vrep.simxGetObjectPosition(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, Pioneer_p3dx, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);






end