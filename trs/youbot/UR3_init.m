function handles = UR3_init(vrep, id)
% Initialize youBot

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,

handles = struct('id', id);

UR_Joints = [-1,-1,-1,-1,-1]; % front left, rear left, rear right, front right
[res UR_Joints(1)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_1', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res UR_Joints(2)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_2', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res UR_Joints(3)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_3', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res UR_Joints(4)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_4', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res UR_Joints(5)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_5', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res UR_Joints(6)] = vrep.simxGetObjectHandle(id, 'Revolute_joint_6', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.UR_Joints = UR_Joints;



[res UR3_tip] = vrep.simxGetObjectHandle(id, 'Tip', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

[res UR3_target] = vrep.simxGetObjectHandle(id, 'Target', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

handles.UR3_tip = UR3_tip;
handles.UR3_target = UR3_target;

for i = 1:6,
  res = vrep.simxGetJointPosition(id, UR_Joints(i), vrep.simx_opmode_streaming); vrchk(vrep, res, true);
end


% Stream the arm joint angles and the tip position/orientation
res = vrep.simxGetObjectPosition(id, UR3_tip, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, UR3_tip, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);

res = vrep.simxGetObjectPosition(id, UR3_target, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, UR3_target, -1, vrep.simx_opmode_streaming); vrchk(vrep, res, true);


end
