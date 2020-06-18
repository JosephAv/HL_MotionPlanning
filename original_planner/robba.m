%% Linear interp

linj1 = linspace(Joint_start(1), Joint_des(1),268) ;
linj2 = linspace(Joint_start(2), Joint_des(2),268) ;
linj3 = linspace(Joint_start(3), Joint_des(3),268) ;
linj4 = linspace(Joint_start(4), Joint_des(4),268) ;
linj4 = linspace(Joint_start(4), Joint_des(4),268) ;
linj5 = linspace(Joint_start(5), Joint_des(5),268) ;
linj6 = linspace(Joint_start(6), Joint_des(6),268) ;
linj7 = linspace(Joint_start(7), Joint_des(7),268) ;
LinJA = [linj1' linj2' linj3' linj4' linj5' linj6' linj7'];


%plotTrajectory(LinJA, KineParameters, P_des, P_start, Joint_start);


figure,plot(Traj_opt)
figure,plot(LinJA)


%% Evaluate Jerk

P_traj_opt = calculatePosefromAnglesTraj(Traj_opt,KineParameters);
P_traj_lin = calculatePosefromAnglesTraj(LinJA,KineParameters);

Pos_opt = P_traj_opt(:,1:3);
Pos_lin = P_traj_lin(:,1:3);

figure,
subplot(1,2,1),plot(Pos_opt)
subplot(1,2,2),plot(Pos_lin)

Pos_opt1 = Pos_opt(:,1);Pos_opt2 = Pos_opt(:,2);Pos_opt3 = Pos_opt(:,3);
Pos_lin1 = Pos_lin(:,1);Pos_lin2 = Pos_lin(:,2);Pos_lin3 = Pos_lin(:,3);

dPos_opt1 = diff(Pos_opt1);dPos_opt2 = diff(Pos_opt2);dPos_opt3 = diff(Pos_opt3);
dPos_lin1 = diff(Pos_lin1);dPos_lin2 = diff(Pos_lin2);dPos_lin3 = diff(Pos_lin3);

dPos_opt = [dPos_opt1 dPos_opt2 dPos_opt3];
dPos_lin = [dPos_lin1 dPos_lin2 dPos_lin3];

figure,
subplot(1,2,1),plot(dPos_opt)
subplot(1,2,2),plot(dPos_lin)

ddPos_opt1 = diff(dPos_opt1);ddPos_opt2 = diff(dPos_opt2);ddPos_opt3 = diff(dPos_opt3);
ddPos_lin1 = diff(dPos_lin1);ddPos_lin2 = diff(dPos_lin2);ddPos_lin3 = diff(dPos_lin3);

ddPos_opt = [ddPos_opt1 ddPos_opt2 ddPos_opt3];
ddPos_lin = [ddPos_lin1 ddPos_lin2 ddPos_lin3];

figure,
subplot(1,2,1),plot(ddPos_opt)
subplot(1,2,2),plot(ddPos_lin)

dddPos_opt1 = diff(ddPos_opt1);dddPos_opt2 = diff(ddPos_opt2);dddPos_opt3 = diff(ddPos_opt3);
dddPos_lin1 = diff(ddPos_lin1);dddPos_lin2 = diff(ddPos_lin2);dddPos_lin3 = diff(ddPos_lin3);

dddPos_opt = [dddPos_opt1 dddPos_opt2 dddPos_opt3];
dddPos_lin = [dddPos_lin1 dddPos_lin2 dddPos_lin3];


figure,
subplot(1,2,1),plot(dddPos_opt)
subplot(1,2,2),plot(dddPos_lin)

figure
subplot(1,2,1),plot(vecnorm(dddPos_opt'))
subplot(1,2,2),plot(vecnorm(dddPos_lin'))