function [traj,rotVel] = rotate(traj,id,h,vrep)
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
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

end

