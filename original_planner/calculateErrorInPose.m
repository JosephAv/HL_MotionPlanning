function P_err = calculateErrorInPose(P_traj, Joint_traj, P_des, Joint_des, P_start, Joint_start)
%this function calculates the error at joint level between the first and
%last frame of the trajectory and the desired initial and final position
%respectively. This function is ready to be used also in cartesian domain.

%P_traj --> matrix of poses during trajectory \in R_N_7
%P_des  --> desired pose \in R_1_7
%P_err  --> matrix of error in pose \in R_N_7

%err_target = P_traj(end,:) - P_des;

% [a,b] = min(sum(( P_traj - P_des)')');
% err_target = P_traj(b,:) - P_des;
err_target = Joint_traj(end,:)-Joint_des;

%err_init = P_traj(1,:) - P_start;
err_init = Joint_traj(1,:)-Joint_start;
% 
% Pos_opt = P_traj(:,1:3);
% 
% Pos_opt1 = Pos_opt(:,1);Pos_opt2 = Pos_opt(:,2);Pos_opt3 = Pos_opt(:,3);
% 
% dPos_opt1 = diff(Pos_opt1);dPos_opt2 = diff(Pos_opt2);dPos_opt3 = diff(Pos_opt3);
% 
% ddPos_opt1 = diff(dPos_opt1);ddPos_opt2 = diff(dPos_opt2);ddPos_opt3 = diff(dPos_opt3);
% 
% dddPos_opt1 = diff(ddPos_opt1);dddPos_opt2 = diff(ddPos_opt2);dddPos_opt3 = diff(ddPos_opt3);
% 
% dddPos_opt = [norm(dddPos_opt1) norm(dddPos_opt2) norm(dddPos_opt3)];
% 




P_err = norm([err_target err_init]); %temporary...to be revised

end

