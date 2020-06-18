function [TotPot] = calculatePotentialObs(Poses, Trajectory, Obstacles, Ndof,KineParameters, ObsModality)
% here we evaluate the potential function

    eta = 1;

    TotPot = 0;
    for i = 1 : size(Obstacles,1)
        ActObs = Obstacles(i,:); %this is a list of shperes defined as [cx cy cz r joint] -- c is the center, r the radius, joint a number 1-4 that selects the joint to which we want obs. avoidance

        % evalDistanceFromObs works in joint domain, with obstacles at joint level
        % evalEEDistanceFromObs works in cartesian space. The function evaluate
        % the mninimum distance between a set of points in 3D space and the end
        % effector of the upper limb during the whole movement
        
        if ObsModality == 1
            [minDistance,~] = evalJointCartesianDistanceFromObs(Trajectory,KineParameters, ActObs); %minDistance is the minimum distance between Traj and Obs.  P_traj and P_Poly are the corresponding points
        else
            [minDistance,~,~] = evalDistanceFromObs(Trajectory, ActObs, Ndof); %minDistance is the minimum distance between Traj and Obs.  P_traj and P_Poly are the corresponding points
        end
        
        
        r = ActObs(4);
        dist = max(minDistance-r,1); % distance is the maximum between minDistance-r and 1 mm  
        TotPot = TotPot + eta/(dist)^2; %sum of inverse distances
    end
end

