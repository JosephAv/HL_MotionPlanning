function [Obs_cart] = getObstaclesCartesianPoints(Obs)
% this function takes as input the list of spheres as [cx cy cz r joint] and gives the points in cartesian space

Obs_cart = [];
for i = 1 : size(Obs,1)
    act_obs = Obs(i,:);
    r = act_obs(4);
    c = act_obs(1:3);
    
    [xs,ys,zs] = sphere;
    xs = r*xs + c(1);
    ys = r*ys + c(2);
    zs = r*zs + c(3);
    Obs_cart = [Obs_cart; reshape([xs,ys,zs],[441,3])];

end




end

