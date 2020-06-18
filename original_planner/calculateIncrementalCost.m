function P_err = calculateIncrementalCost(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs)

    numframes = size(mean_traj,1);
    numdofs = 7;

    k0 = K(1:7);
    Traj_act = repmat(k0,numframes,1) + mean_traj;
    
    numfPCs = length(K)/numdofs - 1;
    for i = 1 : numfPCs %here i calculate how many fPCs i am using and build the correspondent Traj_act
        eval(strcat('fPC',num2str(i),' = fPCs{',num2str(i),'};'));
        eval(strcat('k',num2str(i),' = K( 1+(',num2str(i),')*numdofs : (',num2str(i),'+1)*numdofs);'));
        eval(strcat('Traj_act = Traj_act +  k',num2str(i),'.*fPC',num2str(i),';'));
    end

    P_traj = calculatePosefromAnglesTraj(Traj_act,KineParameters); %calculate EE pose from angles

    P_err = calculateErrorInPose(P_traj, Traj_act, P_des, Joint_des, P_start, Joint_start); %calculate the error
end