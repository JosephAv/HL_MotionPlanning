function [globalmin,P_traj_min,P_Poly] = evalEEDistanceFromObs(PosesEE,CartesianObs)
% this version of the function takes as input the Poses of the robot end
% effector in position and orientation, then evaluate the 3D distance of the EE from
% the obstacles in cartesian space. 

%     if size(PosesEE,1) == 3
%         PosesEE = PosesEE'; % set PosesEE as a Nframes * 3 matrix
%     end




% % %     if size(CartesianObs,1) == 3
% % %         CartesianObs = CartesianObs'; % set CartesianObs as a Nframes * 3 (x y z) matrix
% % %     end
        
% % %     if size(PosesEE,1) > size(CartesianObs,1)
% % %         Big = PosesEE;
% % %         Small = CartesianObs;
% % %     else 
% % %         Big = CartesianObs;
% % %         Small = PosesEE;
% % %     end

% % %     globalmin = Inf;
% % %     for i = 1 : size(Small,2)
% % %         differences = Big-Small(i,:);
% % %         actmin = min(vecnorm( differences')');
% % %         actind = find(vecnorm( differences') == actmin);
% % %         if actmin < globalmin 
% % %             globalmin = actmin;
% % %             P1 = Small(i,:);
% % %             P2 = Big(actind,:);
% % %         end
% % %     end

% % %     if size(PosesEE,2) > size(CartesianObs,2)
% % %         P_traj_min = P2;
% % %         P_Poly = P1;
% % %     else 
% % %         P_traj_min = P1;
% % %         P_Poly = P2;
% % %     end
    
PosesEE = PosesEE(:,1:3); %select only the 3D distance
Big = PosesEE;
Small = [-13 220 350];

differences = Big-Small;
globalmin = min(vecnorm( differences')');
actind = find(vecnorm( differences') == globalmin);
P_traj_min = Big(actind,:);

P_Poly = [];

end

