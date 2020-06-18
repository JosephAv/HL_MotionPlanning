function P_traj = calculatePosefromAnglesTraj(Traj_angles,KineParameters,joint)
%This function calculate the pose of an arbitrary joint (to be selected
%with the input joint) along a trajectory Traj_angles. Traj_angles is a
%matrix of angular values \in R^{Nframes * NDof}. Ouptut is codified as a
%vector, the first 3 values are the position in x y z, the last 4 are the
%quaternion.

if nargin < 3
    joint = 4;
end

    P_traj = zeros(size(Traj_angles,1), 7); %Poses of EE during trajectory

    for i = 1 : size(Traj_angles,1)
        switch joint
            case 1 %shoulder
                f_k_tmp = gWorldS1fun(KineParameters,Traj_angles(i,:));
            case 2 %elbow
                f_k_tmp = gWorldE1fun(KineParameters,Traj_angles(i,:));
            case 3 %wrist
                f_k_tmp = gWorldW1fun(KineParameters,Traj_angles(i,:));
            case 4 %hand
                f_k_tmp = gWorldHfun(KineParameters,Traj_angles(i,:));
        end

        DistWorldHand_tmp = f_k_tmp(1:3,4);
        AngWorldHand_tmp = rotm2quat(f_k_tmp(1:3,1:3));
        P_act = [DistWorldHand_tmp' AngWorldHand_tmp]; %codec: [posx posy posz quat] \in r7

        P_traj(i,:) = P_act;
    end

end

