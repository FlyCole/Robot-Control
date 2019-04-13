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

        [X, Y] = meshgrid(-5:.25:5, -5.5:.25:2.5);
        X = reshape(X, 1, []);
        Y = reshape(Y, 1, []);
    end

    % Make sure everything is settled before we start. 
    pause(2);

    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the state machine. 
    fsm = 'calnav_back';
    t=0;
    rotate_status = 0;
    
    
    % define trash can location
    can_x=[-6.8,-0.8,-6.8,6.8,6.8];
    can_y=[6.25,-0.75,-2.25,2.25,6.25];
    % define object location
    %[theta,obj_x,obj_y]=cal_pos();
    obj_x=[-3.167,-3.35,-3.75,-2.3776,-3.6272];
    obj_y=[-5.25,-5.3065,-6.09,-5.8992,-6.1455];
    theta=[-pi/2,-pi/2,0,0,0];
    
    %% Start the demo. 
    count=1;
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
        angl = theta(count);

        %% Apply the state machine. 
        if strcmp(fsm, 'rotate')
            rotVel = angdiff(angl, youbotEuler(3));
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'drive';
            end
            prevOri = youbotEuler(3);
            
        %% rotate2
        elseif strcmp(fsm, 'rotate2')
            rotVel = angdiff(angl, youbotEuler(3));
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                beta=[pi/4 pi/4 pi/4 pi/1.33 5.14];
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 beta(count)], vrep.simx_opmode_oneshot);
                fsm = 'snapshot';
            end
            prevOri = youbotEuler(3);
          
        %% Then, make it move straight ahead until it reaches the table.
        elseif strcmp(fsm, 'drive')   
            % The further the robot, the faster it drives. (Only check for the first dimension.)
            forwBackVel = -(youbotPos(1) + 3.167);
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevLoc) < .001)
                forwBackVel=0;
                leftRightVel=0;
                rotVel=0;
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 pi/4], vrep.simx_opmode_oneshot);
                fsm = 'snapshot'; 
            end
            prevLoc = youbotPos(1);
            
         %% Read data from the range camera
        elseif strcmp(fsm, 'snapshot')
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            % Then use the depth sensor. 
            fprintf('Capturing point cloud...\n');
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);

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
            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Capturing image...\n');
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

            a=double(image);
            modeblue=mode(a,count);
            
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
             err=error_cal(count);
             res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef,[(pos_1(1)+err(count,1)) pos_1(2)+err(count,2) pos_1(3)+err(count,3)], vrep.simx_opmode_oneshot);
             vrchk(vrep, res, true);

            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % If the arm has reached the wanted position, move on to the next state. 
            zbw=[0.001,0.001,0.001,0.1,0.1];
            if norm(tpos - [(pos_1(1)+err(count,1)) pos_1(2)+err(count,2) pos_1(3)+err(count,3)]) < zbw(count);
                fsm = 'reachout';
            end
        elseif strcmp(fsm, 'reachout')
            %% Move the gripper tip along a line so that it faces the object with the right angle.
            % Get the arm tip position. (It is driven only by this position, except if IK is disabled.)
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);

            if tpos(1) > pos_1(1)
                fsm = 'grasp';
            end
            
            add=[0.01,0.06,0.04,0.13,-0.02];
            
            tpos(1) = tpos(1) + add(count);
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        elseif strcmp(fsm, 'grasp')
            %% Grasp the object by closing the gripper on it.
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
            object_x = can_x(count);
            object_y = can_y(count);
            traj=calnav(object_x,object_y,id,h,vrep);
            fsm = 'rotate1';
   
            %% Rotate to another point after capturing the item
        elseif strcmp(fsm, 'rotate1')
            [traj,rotVel]=rotate(traj,id,h,vrep);
            fsm = 'drive1';
            
        %% Drive after capturing the item
         elseif strcmp(fsm, 'drive1'),
        [traj,target,forwBackVel] = drive(traj,id,h,vrep);
        fsm = 'rotate1';
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
            fsm = 'backoff2';
            
        elseif strcmp(fsm, 'backoff2')
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
                fsm = 'finished';
            end
            
        elseif strcmp(fsm, 'finished')
            pause(3);
            count=count+1;
            fsm = 'turn_back';
        %% Turn back  
        elseif strcmp(fsm, 'turn_back')
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
            angl=[pi*3/2,pi*3/2,pi*3/2,pi/2,pi/2];
            rotVel = angdiff(angl(count-1), youbotEuler(3));
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl(count-1), youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'calnav_back';
            end
           
            prevOri = youbotEuler(3);
       %% calculate map 
        elseif strcmp(fsm, 'calnav_back')
            object_x = obj_x(count);
            object_y = obj_y(count);
            traj=calnav(object_x,object_y,id,h,vrep);
            fsm = 'rotate_back';

       %% Rotate to another point after capturing the item
        elseif strcmp(fsm, 'rotate_back')
            [traj,rotVel]=rotate(traj,id,h,vrep);
            fsm = 'drive_back';
            
        %% Drive after capturing the item
         elseif strcmp(fsm, 'drive_back'),
            [traj,target,forwBackVel] = drive(traj,id,h,vrep);
            fsm = 'rotate_back';
            if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.001  
                forwBackVel=0;
                leftRightVel=0;
                rotVel=0;
                fsm='rotate2';
            end    
         
         %% end
        elseif strcmp(fsm,'end'),
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
    
end % main function
   
