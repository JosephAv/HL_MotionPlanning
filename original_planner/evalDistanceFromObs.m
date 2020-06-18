function [globalmin,P_traj,P_Poly] = evalDistanceFromObs(Traj,Poly, NDof)
% This function takes as input the polytope of an obstacle and an upper limb  
% trajectory in joint domain and evaluate the minimum distance.

    if size(Traj,1) == NDof
        Traj = Traj'; % set Traj as a Nframes * NDof matrix
    end

    if size(Poly,1) == NDof
        Poly = Poly'; % set Poly as a Nframes * NDof matrix
    end

    if size(Traj,1) > size(Poly,1)
        Big = Traj;
        Small = Poly;
    else 
        Big = Poly;
        Small = Traj;
    end

    globalmin = Inf;
    for i = 1 : size(Small,1)
        actmin = min(vecnorm( Big-Small(:,i)));
        actind = find(vecnorm( Big-Small(:,i)) == actmin);
        if actmin < globalmin 
            globalmin = actmin;
            P1 = Small(:,i);
            P2 = Big(:,actind);
        end
    end
    
    if size(Traj,1) > size(Poly,1)
        P_traj = P2;
        P_Poly = P1;
    else 
        P_traj = P1;
        P_Poly = P2;
    end
end

