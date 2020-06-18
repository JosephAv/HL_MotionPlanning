
figure,hold on,
for i = 1:7
    subplot(2,4,i),hold on,plot(EstimatedQ(i,:) )
    
end

for j = 1 : size(Obs_1)
    for i = 1 : 7
    subplot(2,4,i),hold on,plot([1 length(mean_dof1)],[Obs_1(i,j) Obs_1(i,j)],'r' )
    end
end


%% Traj_opt


figure,hold on,
for i = 1:7
    subplot(2,4,i),hold on,plot(diff(Traj_opt_obs(:,i)),'r' )
    
end

hold on,
for i = 1:7
    subplot(2,4,i),hold on,plot(diff(Traj_opt_noobs(:,i)),'g' )
    
end


for j = 1 : size(Obs_2)
    for i = 1 : 7
    subplot(2,4,i),hold on,plot([1 length(Traj_opt)],[Obs_2(j,i) Obs_2(j,i)],'r' )
    end
end
            x_col = [255 0 0]/255;
            y_col = [0 255 0]/255;
            z_col = [0 0 255]/255;
            coord_syst_colors_target = [x_col; y_col; z_col];

for i = 1 : size(Obs_2,1)
    Pose = calculatePosefromAnglesTraj(Obs_2(i,:),KineParameters);
    draw_coordinate_system(50,quat2rotm(Pose(4:7)),Pose(1:3),coord_syst_colors_target) %draw target
end


plotTrajectory(Traj_opt(1:4:end,:), KineParameters, P_des, P_start, Joint_start, Obs_2);

            
    P_err1 = calculateErrorInPose(P_traj_opt, Traj_opt, P_des, Joint_des, P_start, Joint_start); %calculate the error in pose
    
    P_err2 = calculatePotentialObs(P_traj_opt, Traj_opt, Obs, numdofs);