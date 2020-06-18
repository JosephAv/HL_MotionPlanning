close all
clear all
clc

%% Load fPCs

load('Savings/MEAN')
for i = 1 : 7 %7 DoFs
    eval( ['load  Savings/lista_pca_nr_dof',num2str(i),'']) %load structures
    eval( ['scores_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.componenti;']) %load scores
    eval( ['mean_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.media;']) %load mean functions

    eval( ['FD_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.fd;']) %load PCs_struct
    
    for j = 1 : 15 %15 PCs
        eval( ['pc',num2str(j),'_dof',num2str(i),' = FD_dof',num2str(i),'.fPCA{',num2str(j),'};']) %extract single PCs
    end
        
end
clear i j
%figure, plot(pc1_dof1), hold on, plot(pc2_dof1), plot(pc7_dof1)
%legend('pc1','pc2','pc3')

%% Planning %% central frame 517

numframes = 517;
%KineParameters = [180 180 -40  0  40  180 0 20 155 50 5 0 300 225]; 
KineParameters = [125.149108581732,148.431768191511,-115.973666354272,-19.8936354839336,28.6121705580542,158.985333633674,-7.78655708978049,31.1631329834081,155.888434145282,44.9424947316485,14.8290910319249,-8.35699021213840,276.725154360048,241.589264420293];

%%%%%%%% Desired Pose %%%%%%%%
Joint_des = [0 0 0 0 0 0 0];
P_des = calculateActualPose(KineParameters,Joint_des);

% % % AngDes = [0.7071 0 -0.7071 0];%eul2quat([0 0 0]); %from euler angles to pose_quat
% % % %P_des = [100 20 45 AngDes]; %[x y z pose_quat]  pose of end effector \in R7
% % % P_des = [180 185 535 AngDes]; %[x y z pose_quat]  pose of end effector \in R7


%%%%%%%% Initial Pose %%%%%%%%
startingJointValues = [-0.2512 -0.3473 -0.7338 0.1583 -1.1302 0.3126 0.6938];
P_start = calculateActualPose(KineParameters,startingJointValues);
 %P_start =   P_des ;
 
Joint_start = startingJointValues;
%%%%%%%% Actual Pose %%%%%%%% For debug only %%%%%%%%
%Translation Mk Chest-Shoulder, T Shoulder- Mk Elbow, T Elbow- Mk Wrist, T Wrist- Mk Hand, Lenght Arm, Lenght Forearm
AnglesNow = zeros(1,7);
P_act = calculateActualPose(KineParameters,AnglesNow);



%%%%%%%% TMP %%%%%%%% For debug only %%%%%%%%

Joint_start=[-0.3339   -0.2879   -0.5619    0.5961   -0.5394    0.2579    0.6427];
P_start = calculateActualPose(KineParameters,Joint_start);

Joint_des=[-0.1296    0.0135    0.0522    0.6123    0.3480    0.0091    0.0726];
P_des = calculateActualPose(KineParameters,Joint_des);

%%%%%%%% Initialize Opt Trajectory %%%%%%%%
k0 = zeros(1,7); k1 = zeros(1,7); %initialize parameters
k0 = [0.2925   -0.5848   -0.2450    0.7829   -1.5576   -0.0633    0.2769]; %mean(MEAN)
dev_k0 = [0.4653    0.2880    0.2801    0.2881    0.3738    0.3679    0.5485]; %std(MEAN)

mean_traj = [mean_dof1 mean_dof2 mean_dof3 mean_dof4 mean_dof5 mean_dof6 mean_dof7];
mean_traj = mean_traj(250:numframes,:); %select only the reaching movement

% fPC1      = [pc1_dof1 pc1_dof2 pc1_dof3 pc1_dof4 pc1_dof5 pc1_dof6 pc1_dof7]; 
for i = 1 : 15
    eval ( [' fPC',num2str(i),' = [pc',num2str(i),'_dof1 pc',num2str(i),'_dof2 pc',num2str(i),'_dof3 pc',num2str(i),'_dof4 pc',num2str(i),'_dof5 pc',num2str(i),'_dof6 pc',num2str(i),'_dof7]; ' ])
    eval ( [' fPC',num2str(i),'  = fPC',num2str(i),'(250:numframes,:);']) %select only the reaching movement
end

numframes_old = numframes;
numframes = size(mean_traj,1);

%P_err = calculateCost([k0 k1],P_des, P_start, mean_traj, fPC1, numframes, KineParameters);

%% %%%%%% Optimization with 1 fPC %%%%%%%% 
% % % fun = @(K,P_des,P_start,mean_traj,fPC1,numframes, KineParameters) calculateCost(K,P_des,P_start,mean_traj,fPC1,numframes, KineParameters);
% % % 
% % % fun_eval = @(K) fun(K,P_des,P_start,mean_traj,fPC1,numframes, KineParameters);
% % % 
% % % tic
% % % [y_argmin, f_opt, fmincon_flag, fmincon_output] =...
% % %                 fmincon(fun_eval, [k0 k1], [], [], [], [], [k0-dev_k0 -2*ones(1,7)], [k0+dev_k0 +2*ones(1,7)], [],[]);
% % % toc            
% % % 
% % % k0_opt = y_argmin(1:7);
% % % k1_opt = y_argmin(8:end);
% % % % k0_opt = k0;
% % % % k1_opt = y_argmin;
% % % 
% % % Traj_opt = repmat(k0_opt,numframes,1) + mean_traj + k1_opt.*fPC1;
% % % 
% % % P_traj_opt = calculatePosefromAnglesTraj(Traj_opt,KineParameters);


%% %%%%%% Optimization with 3 fPC %%%%%%%% 
k0 = [0.2925   -0.5848   -0.2450    0.7829   -1.5576   -0.0633    0.2769]; %mean(MEAN)
dev_k0 = [0.4653    0.2880    0.2801    0.2881    0.3738    0.3679    0.5485]; %std(MEAN)
k1 = zeros(1,7); k2= zeros(1,7); k3 = zeros(1,7);

P_err = calculateCost([k0 k1 k2 k3],P_des,Joint_des, P_start, Joint_start, mean_traj, fPC1, fPC2, fPC3, numframes, KineParameters);

fun = @(K,P_des,Joint_des,P_start,Joint_start,mean_traj,fPC1, fPC2, fPC3,numframes, KineParameters) calculateCost(K,P_des,Joint_des,P_start,Joint_start,mean_traj,fPC1, fPC2, fPC3,numframes, KineParameters);

fun_eval = @(K) fun(K,P_des,Joint_des,P_start,Joint_start,mean_traj,fPC1, fPC2, fPC3,numframes, KineParameters);

LB = [k0-dev_k0*100 -20*ones(1,7) -0*ones(1,14)];
UB = [k0+dev_k0*100 +20*ones(1,7) 0*ones(1,14)];

 
options = optimoptions('fmincon','MaxFunctionEvaluations',10000,'MaxIterations',10000);


tic
[y_argmin, f_opt, fmincon_flag, fmincon_output] =...
                fmincon(fun_eval, [k0 k1 k2 k3], [], [], [], [], LB, UB, [],options);
toc            

k0_opt = y_argmin(1:7);
k1_opt = y_argmin(8:14);
k2_opt = y_argmin(15:21);
k3_opt = y_argmin(22:28);
% k0_opt = k0;
% k1_opt = y_argmin;

Traj_opt = repmat(k0_opt,numframes,1) + mean_traj + k1_opt.*fPC1 + k2_opt.*fPC2 + k3_opt.*fPC3;

P_traj_opt = calculatePosefromAnglesTraj(Traj_opt,KineParameters);

%% Visualization

plotTrajectory(Traj_opt, KineParameters, P_des, P_start, Joint_start);



