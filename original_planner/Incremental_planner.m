%
% Main script
%

close all
clear all
clc

%% Load fPCs
loadfPCs

addpath fPCA_scripts/kinematics
addpath cads

%% Set Trajectory constraints (initial and final pose)
task = 'drink';
[Joint_des,Joint_start] = setRobotPoses(task); %select initial and final configuration

%%%%%%%% Desired Pose %%%%%%%%
P_des = calculateActualPose(KineParameters,Joint_des); %forward kine

%%%%%%%% Initial Pose %%%%%%%%
P_start = calculateActualPose(KineParameters,Joint_start);

%% importObstacles
Sho = 1;
Elb = 2; 
Wri = 3;
Han = 4;

% set obstacles as ob1 = [c r Han]; where c is the center r is the radius
% and Han selects the joint to which we consider the obstacle avoidance --
% see previous lines.
%remember     PreT = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
% x->y,  y->z, z->x %% to set the center, remember the rotation for the
% visualization tool.
c = [0 220 350]; r = 50; ob1 = [c r Han]; %per ok (T.2)[-13 220 350]
c = [ 70 170 135]; r = 75; ob2 = [c r Elb]; %per ok
c = [ -40 300 -30]; r = 40; ob11 = [c r Elb]; %per ok elbow

c = [ -35 202 310]; r = 40; ob3 = [c r Han]; %per drink (T.4)
c = [ -168 158 322]; r = 40; ob4 = [c r Han]; %per drink

c = [ 313 480 321]; r = 70; ob5 = [c r Han]; %per exultation (T.3)
c = [ -2 311 353]; r = 70; ob6 = [c r Han]; %per exultation
 
c = [ -167 200 323]; r = 90; ob7 = [c r Han]; %per putonleft (T.1) -- r = 40 FOR MULTI -- R = 90 for single obs
c = [ -89 96 436]; r = 40; ob8 = [c r Han]; %per putonleft
c = [ 0 200 100]; r = 40; ob12 = [c r Elb]; %per putonleft elbow

% here we select which obstacles to use for the simulation. Leave Obs = []
% for free motion; Set Obs = [ob1] for single obstacles; Set Obs = [ob3,
% ob4, ...] for multi obstacles.
Obs = [];%[ob4];%[ob1; ob2];[ob7; ob8]
Obs_cart = getObstaclesCartesianPoints(Obs); % returns samples from the sphere (for planning(?) and plotting purposes)

%% Verify Scenario
%this function plots the initial configuration togheter with the obstacles
figure,plotScenarioV2(KineParameters, P_start, Joint_start, P_des, Joint_des, Obs_cart);

%% %%%%%% Incremental Optimization %%%%%%%% 

%%%% define initial conditions %%%%
%set initial condition for k0 as mean among means (:-D)
k0 = [0.2925   -0.5848   -0.2450    0.7829   -1.5576   -0.0633    0.2769]; %mean(MEAN)
dev_k0 = [0.4653    0.2880    0.2801    0.2881    0.3738    0.3679    0.5485]; %std(MEAN)

%load fpcs in a cell structure and set zero conditions for ki
fPCs = {};
for i = 1 : 10 
    eval(strcat('k',num2str(i),' = zeros(1,numdofs);'))
    eval(strcat('fPCs{',num2str(i),'} = fPC',num2str(i),';'))
end

%%%% define cost function %%%%

%cost function
fun = @(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs)...
    calculateIncrementalCostWithObs(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs);

fun_eval = @(K) fun(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs);
 
options = optimoptions('fmincon','MaxFunctionEvaluations',10000,'MaxIterations',10000);

%very large bounds
LB = [k0-dev_k0*100];
UB = [k0+dev_k0*100];

%set inital setup as single synergy optimization
K_ig = [k0];

F_opt_all = [];

for i = 1 : 10
    eval(strcat('k',num2str(i),' = zeros(1,numdofs);'))
end

for numsyns = 1:2%10  %here you may set how many synergies to use as maximum
    disp(['I am trying with  ' num2str(numsyns) ' synergies' ])

    %%%% define bounds %%%%
    LB = [ LB -20*ones(1,7)];
    UB = [ UB +20*ones(1,7)];
    eval(strcat('K_ig = [K_ig k',num2str(numsyns),'];')) %add a new DoF

    %optimize
    tic
    [y_argmin, f_opt, fmincon_flag, fmincon_output] =...
                fmincon(fun_eval, K_ig, [], [], [], [], LB, UB, [],options);
    toc  
    
    %save the cost functions
    F_opt_all = [F_opt_all f_opt];
    
    %save optimal parameters
    for j = 1 : numsyns
        eval(strcat('k',num2str(j),' = y_argmin( 1+(',num2str(j),')*numdofs : (',num2str(j),'+1)*numdofs);'));
    end
    
    K_ig = y_argmin; %comment this line if you want CI=0 for each optimization, otherwise CI are updated with the optimal values of previous step
    
    %exit strategy -- exit if f_opt < 10^-4 or if you dont increase
    %performances for two steps
    if f_opt < 10^-4 || (numsyns>2 && abs(F_opt_all(end-2)-F_opt_all(end))<10^-3)
        disp(['I found a good solution with ' num2str(numsyns) ' synergies and a cost function equal to ' num2str(f_opt) ])
        
        break
    end
    disp(['Solution with ' num2str(numsyns) ' synergies not enough precise: cost function value ' num2str(f_opt) ])
end

%%%%% analyze this optimal solution
num_syns_used = numsyns;
k0_opt = y_argmin(1:7);
Traj_opt = repmat(k0_opt,numframes,1) + mean_traj;
for j = 1 : numsyns
    eval(strcat('k',num2str(j),'_opt = y_argmin( 1+(',num2str(j),')*numdofs : (',num2str(j),'+1)*numdofs);'));
    eval(strcat('Traj_opt = Traj_opt + k',num2str(j),'_opt.*fPC',num2str(j),';'));
end

P_traj_opt = calculatePosefromAnglesTraj(Traj_opt,KineParameters);

P_err_opt = calculateErrorInPose(P_traj_opt, Traj_opt, P_des, Joint_des, P_start, Joint_start);


saveWS %this script saves the workspace with an univoque name which contains the date of the operation
% 
% Traj_opt_6 = Traj_opt;
% 
% save Traj_opt_6 Traj_opt_6

%% Visualization

%figure('units','normalized','outerposition',[0 0 1 1])
%generate video
figure,plotTrajectoryV2(Traj_opt(1:3:end,:), KineParameters, P_des, Joint_des, P_start, Joint_start, Obs_cart, num_syns_used);

%Traj_opt_obs = Traj_opt;

%% Jerk and miscellaneous
%robba %some plots...

%evalAllCases10Syns %perform all the simulations with 1-10 synergies and saves all

%evalHumanLikelinessTrajectories %here we analyze the simulations,
%calculate jerk etc.

%% plot traj and velocities
% Col = {'r','g','b','k','c','m','y'};
% 
% figure,
% for i = 1 : 7
%     subplot(1,1,1);hold on;plot(Traj_opt(:,i),'LineWidth',4)
%     xlabel('Time Cycle')
%     xlim([0 267])
%     xticks([0 66 133 200 267])
%     xticklabels({'0','25','50','75','100'})
%     ylabel('Angle [rad]')
%     set(gca,'FontSize',40)
%     %ylim([-0.02 0.02])
% %     yticks([-0.02 : 0.005 : 0.02])
% %     yticklabels({'-20' '-15' '-10' '-5' '0' '5' '10' '15' '20'})
%     grid on
% end
%     legend('DoF 1', 'DoF 2', 'DoF 3', 'DoF 4', 'DoF 5', 'DoF 6', 'DoF 7')
% 
% % subplot(2,4,i);hold on
% % dim = [0.75 0.2 .8 .3];
% % str = 'Straight Line Plot from 1 to 10 adkdsdkabdjsbdkjasbdjksbkdjasbkjbsdabdkjbskjds';
% % subplot(2,4,i);annotation('textbox',dim,'String',str);
% 
% 
% velocity = diff(Traj_opt);
% figure,
% figure,
% for i = 1 : 7
%     subplot(1,1,1);hold on;plot(velocity(:,i),'LineWidth',4)
%     xlabel('Time Cycle')
%     xlim([0 267])
%     xticks([0 66 133 200 267])
%     xticklabels({'0','25','50','75','100'})
%     ylabel('Joint Velocity [rad/sec]')
%     set(gca,'FontSize',40)
%     %ylim([-0.02 0.02])
% %     yticks([-0.02 : 0.005 : 0.02])
% %     yticklabels({'-20' '-15' '-10' '-5' '0' '5' '10' '15' '20'})
%     grid on
% end
%     legend('DoF 1', 'DoF 2', 'DoF 3', 'DoF 4', 'DoF 5', 'DoF 6', 'DoF 7')
% 
%     
