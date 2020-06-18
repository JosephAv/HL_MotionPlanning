function [globalmin,P_traj_min] = evalJointCartesianDistanceFromObs(Traj_angles,KineParameters,CartesianObs)
% this version of the function takes as input the robot pose in joint
% domain and evaluate the cartesian distance between the obstacle and the
% joint selected. Obstacle is defined as a matrix 1*5, where the first 
% three columns are the coordinates of the sphere center, the 4th column 
% is the sphere radius, the 5th selects the joint to which we perform obstacle avoidance.

joint_ind = CartesianObs(end);
    
PosesEE = calculatePosefromAnglesTraj(Traj_angles,KineParameters,joint_ind); % questa da posizione e quat per ogni frame, joint_ind seleziona a che livello calcolare la cinematica (hand, wrist, elbow, shoulder)

PosesEE = PosesEE(:,1:3); %select only the 3D position of the selected joint

obs_position = CartesianObs(1:3); %this is the center of the sphere

differences = PosesEE-obs_position;

globalmin = min(vecnorm( differences')');
actind = find(vecnorm( differences') == globalmin);
P_traj_min = PosesEE(actind,:);

end

