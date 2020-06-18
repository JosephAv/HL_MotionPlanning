close all
clear all
clc

%% Load fPCs
loadfPCs

addpath fPCA_scripts/kinematics
addpath cads

%%
tasks = {'ok','drink','exultation','putonleft'};

%% importObstacles
Sho = 1;
Elb = 2; 
Wri = 3;
Han = 4;

%remember     PreT = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
% x->y,  y->z, z->x
c = [-13 220 350]; r = 50; ob1 = [c r Han]; %per ok
c = [ 70 170 135]; r = 75; ob2 = [c r Elb]; %per ok

c = [ -35 202 310]; r = 40; ob3 = [c r Han]; %per drink
c = [ -168 158 322]; r = 40; ob4 = [c r Han]; %per drink

c = [ 313 480 321]; r = 70; ob5 = [c r Han]; %per exultation
c = [ -2 311 353]; r = 70; ob6 = [c r Han]; %per exultation

c = [ -167 200 323]; r = 40; ob7 = [c r Han]; %per putonleft
c = [ -89 96 436]; r = 40; ob8 = [c r Han]; %per putonleft


%% Set Trajectory constraints
for tsk = 1 : 4
    task = tasks{tsk};
    
    for obst = 1 : 3
        disp(['Working on task ',num2str(tsk),', obstacles configuration number ', num2str(obst)]);
        
        %% set obstacles
        if obst==1
            Obs = [];
        elseif obst==2
            obnum = 1+(tsk-1)*2;
            obname = ['ob', num2str(obnum)];
            Obs = [eval(obname)];
        elseif obst==3
            obnum = 1+(tsk-1)*2;
            obname1 = ['ob', num2str(obnum)];
            obname2 = ['ob', num2str(obnum+1)];
            Obs = [eval(obname1); eval(obname2)];
        end
        Obs_cart = getObstaclesCartesianPoints(Obs);

        [Joint_des,Joint_start] = setRobotPoses(task);

        %%%%%%%% Desired Pose %%%%%%%%
        P_des = calculateActualPose(KineParameters,Joint_des);

        %%%%%%%% Initial Pose %%%%%%%%
        P_start = calculateActualPose(KineParameters,Joint_start);

        %% %%%%%% Incremental Optimization %%%%%%%% 

        %%%% define initial conditions %%%%
        k0 = [0.2925   -0.5848   -0.2450    0.7829   -1.5576   -0.0633    0.2769]; %mean(MEAN)
        dev_k0 = [0.4653    0.2880    0.2801    0.2881    0.3738    0.3679    0.5485]; %std(MEAN)

        fPCs = {};
        for i = 1 : 10 
            eval(strcat('k',num2str(i),' = zeros(1,numdofs);'))
            eval(strcat('fPCs{',num2str(i),'} = fPC',num2str(i),';'))
        end

        %%%% define cost function %%%%

        fun = @(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs)...
            calculateIncrementalCostWithObs(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs);

        fun_eval = @(K) fun(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs);

        options = optimoptions('fmincon','MaxFunctionEvaluations',10000,'MaxIterations',10000);

        LB = [k0-dev_k0*100];
        UB = [k0+dev_k0*100];
        K_ig = [k0];

        F_opt_all = [];

        for i = 1 : 10
            eval(strcat('k',num2str(i),' = zeros(1,numdofs);'))
        end

        for numsyns = 1:10 
            disp(['I am trying with  ' num2str(numsyns) ' synergies' ])

            %%%% define bounds %%%%
            LB = [ LB -20*ones(1,7)];
            UB = [ UB +20*ones(1,7)];
            eval(strcat('K_ig = [K_ig k',num2str(numsyns),'];')) %add a new DoF

            tic
            [y_argmin, f_opt, fmincon_flag, fmincon_output] =...
                        fmincon(fun_eval, K_ig, [], [], [], [], LB, UB, [],options);
            toc  

            F_opt_all = [F_opt_all f_opt];
            for j = 1 : numsyns
                eval(strcat('k',num2str(j),' = y_argmin( 1+(',num2str(j),')*numdofs : (',num2str(j),'+1)*numdofs);'));
            end
            %K_ig = y_argmin; %comment this line if you want CI=0 for each optimization, otherwise CI are updated with the optimal values of previous step

            if f_opt < 10^-7 %|| (numsyns>2 && abs(F_opt_all(end-2)-F_opt_all(end))<10^-6)
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

        saveWS
    end
end

load handel
sound(y,Fs)