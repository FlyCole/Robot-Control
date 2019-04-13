function traj = calnav(object_x,object_y,id,h,vrep)
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    % House map. Has 0 for free space and 1 for obstacles.
    load('map');
    % House map, with all obstacles dilated by one extra cell.
    load('fmap');
    cellsize = 0.25;
    
    object_x
    object_y
    
    goal = [ ij(object_x), ij(object_y) ];
    dx = DXform(double(map), 'private');
    dx.plan([goal(1); goal(2)]);

    traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';
    traj = [ xy(traj(1,:)); xy(traj(2,:))];
    traj = [traj [object_x;object_y]];

    figure(3);

    [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
    plot(X(map==1), Y(map==1), '*r', 7.5, 0, 'or', 0, 7.5, 'og',traj(1,:), traj(2,:), 'b');
end

