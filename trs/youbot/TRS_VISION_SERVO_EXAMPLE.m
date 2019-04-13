function youbot_vision()
    % youbot Illustrates the V-REP Matlab bindings.


    close all;
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % If you get an error like: 
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time
    % we try to get a joint angle or the robot pose with the simx_opmode_buffer
    % option.
    pause(.2);

    %% Youbot constants
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm.
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

    %% Preset values for the demo. 
    disp('Starting robot');
    
    % Define the preset pickup pose. 
%    pickupJoints = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];

    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;

    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value and exit in case of error. 
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);

    % Initialise the plot. 
    plotData = true;
    if plotData
        subplot(211);
        drawnow;
        
        % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
        % see, by selecting the points within this mesh that are within the visibility range. 
        [X, Y] = meshgrid(-5:.25:5, -5.5:.25:2.5);
        X = reshape(X, 1, []);
        Y = reshape(Y, 1, []);
    end

    % Make sure everything is settled before we start. 
    pause(2);

    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the state machine. 
    fsm = 'rotate';
    t=0;
    rotate_status = 0;
    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        fsm
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Plot something if required. 
        if plotData
            % Read data from the Hokuyo sensor.
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);

            % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo. 
            in = inpolygon(X, Y, [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
                          [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);

            % Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
            % visibility range from the Hokuyo sensor. 
            % The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
            % sensor position. 
            subplot(211)
            plot(X(in), Y(in), '.g',...
                 pts(1, contacts), pts(2, contacts), '*r',...
                 [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)], [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)], 'r',...
                 0, 0, 'ob',...
                 h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or',...
                 h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
            axis([-5.5, 5.5, -5.5, 2.5]);
            axis equal;
            drawnow;
        end
        angl = -pi/2;

        %% Apply the state machine. 
        if strcmp(fsm, 'rotate')
            %% First, rotate the robot to go to one table.             % The rotation velocity depends on the difference between the current angle and the target. 
            rotVel = angdiff(angl, youbotEuler(3));
            
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'drive';
            end
           
            prevOri = youbotEuler(3);
        elseif strcmp(fsm, 'drive')
            %% Then, make it move straight ahead until it reaches the table. 
            % The further the robot, the faster it drives. (Only check for the first dimension.)
            forwBackVel = -(youbotPos(1) + 3.167);

            % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
            % a specific location before moving on to the next state.
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevLoc) < .001)
                forwBackVel = 0;
                
                % Change the orientation of the camera
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 pi/4], vrep.simx_opmode_oneshot);
                %%%%%%%%%%%%%%%%%%% Move the arm to the preset pose.


                fsm = 'snapshot';
               
            end
            prevLoc = youbotPos(1);
        elseif strcmp(fsm, 'snapshot')
            %% Read data from the range camera
            % Reading a 3D image costs a lot to VREP (it has to simulate the image). It
            % also requires a lot of bandwidth, and processing a 3D point cloud (for
            % instance, to find one of the boxes or cylinders that the robot has to
            % grasp) will take a long time in MATLAB. In general, you will only want to
            % capture a 3D image at specific times, for instance when you believe you're
            % facing one of the tables.

            % Reduce the view angle to better see the objects. Indeed, the number of rays the Hokuyo sends is limited;
            % if this number is used on a smaller angle, then the resolution is better. 
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE, and turn itself off again. 
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Then use the depth sensor. 
            fprintf('Capturing point cloud...\n');
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
            % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
            % the output data. To get a correct plot, you should invert the y and z dimensions. 
    
            % Here, we only keep points within 1 meter, to focus on the table. 
            pts = pts(1:3, pts(4, :) < 1);

            if plotData
                subplot(223)
                plot3(pts(1, :), pts(3, :), pts(2, :), '*');
                axis equal;
                view([-169 -46]);
            end

            % Save the pointcloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
            fileID = fopen('pc.xyz','w');
            fprintf(fileID,'%f %f %f\n',pts);
            fclose(fileID);
            fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));

            %% Read data from the RGB camera
            % This is very similar to reading from the 3D camera. The comments in the 3D camera section directly apply 
            % to this section.

            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Capturing image...\n');
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

            a=double(image);
            modeblue=zeros(size(a(:,:,1)));
            modeblue=(a(:,:,1)>150).*(a(:,:,2)>150).*(a(:,:,3)<100);
            a(:,:,1)=255*modeblue;
            a(:,:,2)=255*modeblue;
            a(:,:,3)=255*modeblue;
            figure,imshow(uint8(a));
            
            b=rgb2gray(a);
            bw=im2bw(b);
            [r c]=find(bw==1);
            [rectx,recty,area,perimeter] = minboundrect(c,r,'a'); % 'a'是按面积算的最小矩形，如果按边长用'p'。
            figure;imshow(bw);
            line(rectx(:),recty(:),'color','r');
            
            center = ceil([(rectx(1,1)+rectx(2,1))/2,(recty(2,1)+recty(3,1))/2]);
            
            center_real = (center - 256)/1600;
            center_real(2)=-center_real(2);
            
            zs = find((pts(1,:)-center_real(1))<0.05& ...
                      (pts(1,:)-center_real(1))>-0.05& ...
                      (pts(2,:)-center_real(2))< 0.05& ...
                      (pts(2,:)-center_real(2))>-0.05); 
            
            z = sum(pts(3,zs))/size(zs,2);
            y = sum(pts(2,zs))/size(zs,2);
            x = sum(pts(1,zs))/size(zs,2);
            
            [res, rgbdp]=vrep.simxGetObjectPosition(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  
            [res, rgbdpo]=vrep.simxGetObjectOrientation(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  

            T =transl(rgbdp)*trotx(rgbdpo(1))*troty(rgbdpo(2))*trotz(rgbdpo(3));

            pos_1 =T* [x;y;z;1]
            
            if plotData
                subplot(224)
                imshow(image);
                drawnow;
            end

            tic;  
            for t = 0.001:0.001:2  
                while toc < t  
                end  
            end
            % Next state. 
            fsm = 'extend';
        elseif strcmp(fsm, 'extend')
            %% Move the arm to face the object.
             [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
             vrchk(vrep, res, true);
             res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
             vrchk(vrep, res, true);
             res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef,[(pos_1(1)-0.07) pos_1(2)+0.02 pos_1(3)-0.03], vrep.simx_opmode_oneshot);
             vrchk(vrep, res, true);
%                             for i = 1:5
%                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
%                                                           vrep.simx_opmode_oneshot);
%                     vrchk(vrep, res, true);
%                 end
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % If the arm has reached the wanted position, move on to the next state. 
            if norm(tpos - [(pos_1(1)-0.07) pos_1(2)+0.02 pos_1(3)-0.03]) < 0.002
            %if norm(tpos - [0.3259 -0.001 0.2951]) < .002
                % Set the inverse kinematics (IK) mode to position AND orientation. 
 %               res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
 %              vrchk(vrep, res, true);
                fsm = 'reachout';
            end
        elseif strcmp(fsm, 'reachout')
            %% Move the gripper tip along a line so that it faces the object with the right angle.
            % Get the arm tip position. (It is driven only by this position, except if IK is disabled.)
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);

            % If the tip is at the right position, go on to the next state. 
            if tpos(1) > pos_1(1)
                fsm = 'grasp';
            end

            % Move the tip to the next position (it moves along a line). 
            tpos(1) = tpos(1) + .01;
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        elseif strcmp(fsm, 'grasp')
            %% Grasp the object by closing the gripper on it.
            % Close the gripper. Please pay attention that it is not possible to determine the force to apply and 
            % object will sometimes slips from the gripper!
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(2);
            
            % Disable IK; this is used at the next state to move the joints manually. 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fsm = 'backoff';
        elseif strcmp(fsm, 'backoff')
            %% Go back to rest position.
            % Set each joint to their original angle. 
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            
            % Get the gripper position and check whether it is at destination.
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            %%if norm(tpos - homeGripperPosition) < .02
            %%    % Open the gripper. 
            %%    res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            %%    vrchk(vrep, res);
            %%end
            
            if norm(tpos - [homeGripperPosition(1) homeGripperPosition(2) homeGripperPosition(3)]) < .002
                fsm = 'calnav';
            end
           %%%if 
        elseif strcmp(fsm, 'calnav')
            object_x = -6.8;
            object_y = -2.25;
            
            [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
        
            % House map. Has 0 for free space and 1 for obstacles.
            load('map');
            % House map, with all obstacles dilated by one extra cell.
            load('fmap');
            cellsize = 0.25;
            
            goal = [ ij(object_x), ij(object_y) ];
            dx = DXform(double(fmap), 'private');
            dx.plan([goal(1); goal(2)]);
            
            traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';
            traj = [ xy(traj(1,:)); xy(traj(2,:))];
            traj = [traj [-6.8;-2.25]];
            
            figure(3);
           
            [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
            plot(X(map==1), Y(map==1), '*r', 7.5, 0, 'or', 0, 7.5, 'og',traj(1,:), traj(2,:), 'b');

            fsm = 'rotate1';

       
            %% Rotate to another point after capturing the item
        elseif strcmp(fsm, 'rotate1')
              target = traj(:,1);
                
              sqrt((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2)
              
              if size(traj, 2) > 1 & sqrt((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2)<0.5 %mod(int32(1000*t),500) == 0
                  traj = traj(:,2:end);
              end
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            P_ow=[target(1);target(2);1];
            T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
            P_oc=inv(T_cw)*P_ow;  %get the object position relative to the youbot_center coordinate
            if P_oc(1)>0
                 angle=atan(P_oc(2)/P_oc(1));
                angl=angle-pi/2;   
            end
            if P_oc(1)<0
                angle=atan(P_oc(2)/P_oc(1));
                angl=angle+pi/2;
            end
            if P_oc(1)==0
                angl=0;
            end

            rotVel =angl;

            fsm = 'drive1';
% 		if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
%             rotVel = 0;
%         end
 		%forwBackVel=0;
        %% Drive after capturing the item
         elseif strcmp(fsm, 'drive1'),
         target = traj(:,1);
  
              if size(traj, 2) > 1 & sqrt((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2)<0.5 %mod(int32(1000*t),500) == 0
                  traj = traj(:,2:end);
              end
        forwBackVel = sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)/2;
		%forwBackVel=forwBackVel;
        fsm = 'rotate1';
%         if abs(forwBackVel) < .001,
%             forwBackVel = 0;
%         end
%         
 		 %rotVel=0;
         if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.01
             fsm='drop';
             forwBackVel=0;
             leftRightVel=0;
             rotVel=0;
         end
 

            %% Drop after reaching the destbin       
            %% Extend the arm after arriving the destbin
        elseif strcmp(fsm, 'drop')
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % If the arm has reached the wanted position, move on to the next state. 
            %if norm(tpos - [pos_l(1), pos_l(2), pos_l(3)]) < .002
            aimpos = [-0.5 0.4 0.4]
            %dist is the distance to the destbin
            dist=sqrt((tpos(1)-(-0.2))^2+(tpos(2)-(0.2))^2+(tpos(3)-(0.2))^2)
            
            % Set the inverse kinematics (IK) mode to position AND orientation. 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, aimpos, vrep.simx_opmode_oneshot);
            
            if dist < .3
                fsm= 'release';
            end
            
            %% Release the item after reaching the upward of destbin
        elseif strcmp(fsm, 'release')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(2);
            
            % Disable IK; this is used at the next state to move the joints manually. 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fsm = 'finished';
 
        elseif strcmp(fsm, 'finished')
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
%           res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
%           res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),...
%                                           -forwBackVel-leftRightVel+rotVel,...
%                                           vrep.simx_opmode_oneshot); vrchk(vrep, res);
%           res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),...
%                                           -forwBackVel+leftRightVel+rotVel,...
%                                           vrep.simx_opmode_oneshot); vrchk(vrep, res);
%           res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(3),...
%                                           -forwBackVel-leftRightVel-rotVel,...
%                                           vrep.simx_opmode_oneshot); vrchk(vrep, res);
%           res = vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(4),...
%                                           -forwBackVel+leftRightVel-rotVel,...
%                                           vrep.simx_opmode_oneshot); vrchk(vrep, res);
%           res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);
        
        %vrep.simxSynchronousTrigger(id);
        %t = t+timestep;
        % Make sure that we do not go faster that the simulator (each iteration must take 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
    
end % main function
   
