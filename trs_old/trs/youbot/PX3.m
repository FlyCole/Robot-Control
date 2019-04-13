function PX3()
% youbot Illustrates the V-REP Matlab bindings.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
   
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
   h = PX3_init(vrep, id);

   
   
    % Update wheel velocities
   res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
   vrep.simxSetJointTargetVelocity(id, h.wheelJoints(1),...
                                   1,...
                                   vrep.simx_opmode_oneshot); vrchk(vrep, res);
   vrep.simxSetJointTargetVelocity(id, h.wheelJoints(2),...
                                   1,...
                                   vrep.simx_opmode_oneshot); vrchk(vrep, res);
   res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);
end

