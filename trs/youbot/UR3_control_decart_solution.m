function UR3_control_decart()
 
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
T=20;
N_step=T/timestep;

[tposx,tposxd,tposxdd]=tpoly(0.24933,-3,N_step);
[tposy,tposyd,tposydd]=tpoly(0.26086,-5.65,N_step);
[tposz,tposzd,tposzdd]=tpoly(0.45636,0.2451,N_step);
tpos=[tposx tposy tposz];

[tposex,tposexd,tposexdd]=tpoly(-90*pi/180,-90*pi/180,N_step);
[tposey,tposeyd,tposeydd]=tpoly(70*pi/180,70*pi/180,N_step);
[tposez,tposezd,tposezdd]=tpoly(130*pi/180,130*pi/180,N_step);
tpose=[tposex tposey tposez];

[tposx1,tposx1d,tposx1dd]=tpoly(0.3,0.24933,N_step);
[tposy1,tposy1d,tposy1dd]=tpoly(0.2,0.26086,N_step);
[tposz1,tposz1d,tposz1dd]=tpoly(0.05,0.45636,N_step);
tpos1=[tposx1 tposy1 tposz1];

[tposex1,tposex1d,tposex1dd]=tpoly(-90*pi/180,-90*pi/180,N_step);
[tposey1,tposey1d,tposey1dd]=tpoly(70*pi/180,70*pi/180,N_step);
[tposez1,tposez1d,tposez1dd]=tpoly(130*pi/180,130*pi/180,N_step);
tpose1=[tposex1 tposey1 tposez1];

i=1; 	

fsm='reach';
i=1;
while true
    tic
    if vrep.simxGetConnectionId(id) == -1,
       error('Lost connection to remote API.');
    end
    [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.UR3_target, -1, vrep.simx_opmode_buffer);
    youbotEuler;
    if strcmp(fsm,'reach')
        res = vrep.simxSetObjectPosition(id, h.UR3_target, -1, tpos(i,:), vrep.simx_opmode_oneshot);
        res = vrep.simxSetObjectOrientation(id, h.UR3_target, -1, tpose(i,:), vrep.simx_opmode_oneshot);
        i=i+1;
        if i<N_step
            fsm='reach';
        else
            fsm='back';
            pause(2);
            i=1;
        end
    elseif strcmp(fsm,'back')
       res = vrep.simxSetObjectPosition(id, h.UR3_target, -1, tpos1(i,:), vrep.simx_opmode_oneshot);
       res = vrep.simxSetObjectOrientation(id, h.UR3_target, -1, tpose1(i,:), vrep.simx_opmode_oneshot);
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

   elapsed = toc;
   timeleft = timestep-elapsed;
   if (timeleft > 0),
        pause(min(timeleft, .01));
   end
   i=i+1;
end
  
%pause(5);   
end % main function
   
