function [traj,target,forwBackVel] = drive(traj,id,h,vrep)
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    target = traj(:,1);
    if size(traj, 2) > 1 & sqrt((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2)<0.5 %mod(int32(1000*t),500) == 0
      traj = traj(:,2:end);
    end
    forwBackVel = sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)/2;
end

