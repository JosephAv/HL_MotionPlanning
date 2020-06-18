function P_err = calculateIncrementalCostWithObs(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obstacles)
%this function calculate the cost value

    % Obstacles is a vector of cells. Each cell contains the points in
    % joint space of a polytope
    
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

    P_traj = calculatePosefromAnglesTraj(Traj_act,KineParameters); %forward kine on the whole trajectory: calculate EE pose (Cartesian coordinates + quaternion) from angles


    P_err1 = calculateErrorInPose(P_traj, Traj_act, P_des, Joint_des, P_start, Joint_start); %calculate the error in POSE!

    ObsModality = 1; %set ObsModality = 1 if obstacles are spheres in R3, any other value otherwise
    
    %Obstacles is a matrix N*5, where N is the number of obstacles, the
    %first three columns are the coordinates of the sphere center, the 4th
    %column is the sphere radius, the 5th selects the joint to which we
    %perform obstacle avoidance.

    P_err2 = calculatePotentialObs(P_traj, Traj_act, Obstacles, numdofs, KineParameters, ObsModality);


    alpha1 = 1; alpha2 = 10;
    P_err = alpha1*P_err1 + alpha2*P_err2; % weighted sum of potential and error in pose cost function

end