function P_err = calculateCost(K, P_des, Joint_des, P_start, Joint_start, mean_traj, fPC1, fPC2, fPC3, numframes, KineParameters)

    k0 = K(1:7); k1 = K(8:14); k2 = K(15:21); k3 = K(22:28);
    
    Traj_act = repmat(k0,numframes,1) + mean_traj + k1.*fPC1 + k2.*fPC2 + k3.*fPC3; %buld Traj with 1 fPC -- these are angles

    P_traj = calculatePosefromAnglesTraj(Traj_act,KineParameters); %calculate EE pose from angles

    %%%%%%%% Calculate Error %%%%%%%% For debug only %%%%%%%%

    P_err = calculateErrorInPose(P_traj, Traj_act, P_des, Joint_des, P_start, Joint_start);
    %min(P_err)

end