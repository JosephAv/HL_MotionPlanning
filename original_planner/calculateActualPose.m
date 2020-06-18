function P_act = calculateActualPose(KineParameters,AnglesNow)
%forward kinematics -- from joint angles and kinematic parameters to pose
%and orientation of the hand

THand = fkine_hand(KineParameters, AnglesNow); %forward kinematics
DistWorldHand = THand(1:3,4);
AngWorldHand = rotm2quat(THand(1:3,1:3));
P_act = [DistWorldHand' AngWorldHand]; %Actual pose \in R7


end

