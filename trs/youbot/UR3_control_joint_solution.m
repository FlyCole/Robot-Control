function UR3_control_joint()
 
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
   h = UR3_init(vrep, id);
   pause(.2);
   
   % Constants:
   

  
   disp('Starting robot');
   timestep = .05;
T=10;
N_step=T/timestep;

[joint1,joint1d,joint1dd]=tpoly(0*pi/180,20*pi/180,N_step);
[joint2,joint2d,joint2dd]=tpoly(30*pi/180,30*pi/180,N_step);
[joint3,joint3d,joint3dd]=tpoly(0*pi/180,40*pi/180,N_step);
[joint4,joint4d,joint4dd]=tpoly(0*pi/180,20*pi/180,N_step);
[joint5,joint5d,joint5dd]=tpoly(30*pi/180,30*pi/180,N_step);
[joint6,joint6d,joint6dd]=tpoly(0*pi/180,90*pi/180,N_step);

[joint11,joint11d,joint11dd]=tpoly(90*pi/180,0*pi/180,N_step);
[joint22,joint22d,joint22dd]=tpoly(30*pi/180,30*pi/180,N_step);
[joint33,joint33d,joint33dd]=tpoly(40*pi/180,0*pi/180,N_step);
[joint44,joint44d,joint44dd]=tpoly(20*pi/180,0*pi/180,N_step);
[joint55,joint55d,joint55dd]=tpoly(30*pi/180,30*pi/180,N_step);
[joint66,joint66d,joint66dd]=tpoly(90*pi/180,0*pi/180,N_step);


res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);

i=1; 	
t=0;
fsm='reach';
i=1;
%res = vrep.simxSetIntegerSignal(id, 'ik_mode', 0, vrep.simx_opmode_oneshot_wait);
while true
    tic
    if vrep.simxGetConnectionId(id) == -1,
       error('Lost connection to remote API.');
    end

    if strcmp(fsm,'reach')
       res = vrep.simxSetJointPosition(id, h.UR_Joints(1), joint1(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(2), joint2(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(3), joint3(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(4), joint4(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(5), joint5(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(6), joint6(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
%         [res tippos]= vrep.simxGetObjectPosition(id, h.UR3_tip, -1, vrep.simx_opmode_buffer); vrchk(vrep, res, true);
%         [res tippose]= vrep.simxGetObjectOrientation(id, h.UR3_tip, -1, vrep.simx_opmode_buffer); vrchk(vrep, res, true);
%         res = vrep.simxSetObjectPosition(id, h.UR3_target, -1,tippos,vrep.simx_opmode_oneshot);
%         res = vrep.simxSetObjectOrientation(id, h.UR3_target, -1,tippose,vrep.simx_opmode_oneshot);
        i=i+1;
        if i<N_step
            fsm='reach';
        else
            fsm='back';
           % res = vrep.simxSetIntegerSignal(id, 'ik_mode', 2, vrep.simx_opmode_oneshot_wait);
%              [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.UR3_target, -1, vrep.simx_opmode_buffer);
%              [res youbotpos] = vrep.simxGetObjectPosition(id, h.UR3_target, -1, vrep.simx_opmode_buffer);
            pause(2);
            i=1;
        end
    elseif strcmp(fsm,'back')
%         youbotpos(1)=youbotpos(1)+0.001
%         %youbotpos(1)=youbotpos(1)+0.001;
%         res = vrep.simxSetObjectPosition(id, h.UR3_target, -1, youbotpos, vrep.simx_opmode_oneshot);
%         res = vrep.simxSetObjectOrientation(id, h.UR3_target, -1, youbotEuler, vrep.simx_opmode_oneshot);
       res = vrep.simxSetJointPosition(id, h.UR_Joints(1), joint11(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(2), joint22(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(3), joint33(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(4), joint44(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(5), joint55(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        res = vrep.simxSetJointPosition(id, h.UR_Joints(6), joint66(i),vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
       i=i+1;
        if i<N_step
            fsm='back';
        else
            fsm='finished';
            pause(2);
            i=1;
        end
    elseif strcmp(fsm,'finished')

            break;
    end
  vrep.simxSynchronousTrigger(id);
  t = t+timestep;
end

   elapsed = toc;
   timeleft = timestep-elapsed;
   if (timeleft > 0),
        pause(min(timeleft, .01));
   end
   i=i+1;
end
  

   
