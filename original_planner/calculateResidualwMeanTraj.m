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


P_err_Zero = [];
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
    
        K = zeros(1,7);
        
        fPCs = {};
        for i = 1 : 10 
            eval(strcat('k',num2str(i),' = zeros(1,numdofs);'))
            eval(strcat('fPCs{',num2str(i),'} = fPC',num2str(i),';'))
        end
        P_err = calculateIncrementalCostWithObs(K, P_des, Joint_des, P_start, Joint_start, mean_traj, KineParameters, fPCs, Obs);
        P_err_Zero = [P_err_Zero P_err];
    end
end

save Data/P_err_Zero P_err_Zero
