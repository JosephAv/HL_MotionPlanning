close all
clear all
clc

load fPCA_scripts/UpperLimbParametersDEFAndrea

namefile = 'task_drink_2s_2obs_Ash';
savefolder = 'csv/';
load(['Ashwin/', namefile])

addpath fPCA_scripts/kinematics/

%%% load
EE_T = [];
for i = 1 : size(Traj_opt,1)
    EE_T(:,:,i) = gWorldW1fun(UpperLimbParametersDEF,Traj_opt(i,:));
end

EL_T = [];
for i = 1 : size(Traj_opt,1)
    EL_T(:,:,i) = gWorldE1fun(UpperLimbParametersDEF,Traj_opt(i,:));
end

%%% reshape

EE_T_vector = [];
for i = 1 : size(EE_T,3)
    EE_T_vector = [EE_T_vector; reshape(EE_T(:,:,i),1,16)];
end

csvwrite([savefolder,namefile,'_RT_wrist'],EE_T_vector)


EL_T_vector = [];
for i = 1 : size(EL_T,3)
    EL_T_vector = [EL_T_vector; reshape(EL_T(:,:,i),1,16)];
end

csvwrite([savefolder,namefile,'_RT_elbow'],EL_T_vector)

%%% pos orient
EE_pos = [];
for i = 1 : size(EE_T,3)
    EE_pos = [EE_pos; reshape(EE_T(1:3,4,i),1,3)];
end

EE_quat = [];
for i = 1 : size(EE_T,3)
    myrotm = EE_T(1:3,1:3,i);
    myquat = rotm2quat(myrotm);
    EE_quat = [EE_quat; [myquat(2:4) myquat(1)] ];
end
EE_p_q_vect = [EE_pos EE_quat];

csvwrite([savefolder,namefile,'_cartesian_pos_quat'],EE_p_q_vect)

%%% plots
% 
% 
figure,plot3(squeeze(EE_T(1,4,:)),squeeze(EE_T(2,4,:)),squeeze(EE_T(3,4,:)))
hold on,plot3(squeeze(EL_T(1,4,:)),squeeze(EL_T(2,4,:)),squeeze(EL_T(3,4,:)))


figure,subplot(1,3,1),plot(squeeze(EE_T(1,4,:)))
subplot(1,3,2),plot(squeeze(EE_T(2,4,:)))
subplot(1,3,3),plot(squeeze(EE_T(3,4,:)))

%%