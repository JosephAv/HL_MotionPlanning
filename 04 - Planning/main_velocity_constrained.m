%Del tutto analogo a main_planning con la differenza che in questo caso
%vengono esplicitament vincolate a zero le velocità iniziali e finali anche
%nel caso di assenza di ostacoli

close all
clear all
clc

carica_pc;

lw = 502;
up = 757;
l = up-lw+101;

media_dof1 = media_dof1(lw:up); 
pc1_dof1 = pc1_dof1(lw:up);
pc2_dof1 = pc2_dof1(lw:up);
pc3_dof1 = pc3_dof1(lw:up);
pc4_dof1 = pc4_dof1(lw:up);
pc5_dof1 = pc5_dof1(lw:up);

media_dof2 = media_dof2(lw:up); 
pc1_dof2 = pc1_dof2(lw:up);
pc2_dof2 = pc2_dof2(lw:up);
pc3_dof2 = pc3_dof2(lw:up);
pc4_dof2 = pc4_dof2(lw:up);
pc5_dof2 = pc5_dof2(lw:up);

media_dof3 = media_dof3(lw:up); 
pc1_dof3 = pc1_dof3(lw:up);
pc2_dof3 = pc2_dof3(lw:up);
pc3_dof3 = pc3_dof3(lw:up);
pc4_dof3 = pc4_dof3(lw:up);
pc5_dof3 = pc5_dof3(lw:up);

media_dof4 = media_dof4(lw:up); 
pc1_dof4 = pc1_dof4(lw:up);

media_dof5 = media_dof5(lw:up); 
pc1_dof5 = pc1_dof5(lw:up);

media_dof6 = media_dof6(lw:up); 
pc1_dof6 = pc1_dof6(lw:up);

dof1 = [media_dof1, pc1_dof1, pc2_dof1, pc3_dof1, pc4_dof1, pc5_dof1];
dof2 = [media_dof2, pc1_dof2, pc2_dof2, pc3_dof2, pc4_dof2, pc5_dof2];
dof3 = [media_dof3, pc1_dof3, pc2_dof3, pc3_dof3, pc4_dof3, pc5_dof3];
dof4 = [media_dof4, pc1_dof4];
dof5 = [media_dof5, pc1_dof5];
dof6 = [media_dof6, pc1_dof6];

dof = cell(6,1);
dof{1} = dof1;
dof{2} = dof2;
dof{3} = dof3;
dof{4} = dof4;
dof{5} = dof5;
dof{6} = dof6;

%% Definizione Inizio e Fine

inizio = [-200 400 400 0];
fine = [-200 -400 400 5];
ori_inizio = [0 0 -pi/2 0];
ori_fine = [0 0 pi/2 5];
v_start = 0;
v_finish = 0;

l = length(media_dof1);

%% Definizione Ostacoli

% obs = [-270 -100 400 70;...
%        -130 -100 400 70;...
%        -200 -100 400 70];

% obs = [0 0 300 80];

obs = [];


%% Posizione
offset = 50;
tic
traiettoria_diretta = [];
t_posizione = [];
viapoint = [];

for i=1:3
    [traiettoria, t] = traj_no_obs_new(dof{i}, inizio(i), fine(i), fine(4), v_start, v_finish);
    traiettoria_diretta = [traiettoria_diretta traiettoria];
    t_posizione = [t_posizione t];
end

if size(obs,1)~=0
    if traj_check_collision(traiettoria_diretta, obs, offset)
        [traiettoria_posizione, t_posizione, viapoint] = calcolo_traj(inizio(1:3),inizio(4),fine(1:3),fine(4),obs,offset,dof,zeros(1,3),zeros(1,3));
    else
        traiettoria_posizione = traiettoria_diretta;
    end
else
    traiettoria_posizione = traiettoria_diretta;
end



%% Orientazione

traiettoria_orientazione = [];
t_orientazione = [];

for i=4:6
    [traiettoria, t] = traj_no_obs(dof{i}, ori_inizio(i-3), ori_fine(i-3), fine(4));
    traiettoria_orientazione = [traiettoria_orientazione traiettoria];
    t_orientazione = [t_orientazione t+ori_inizio(4)];
end


%% Resampling

if length(traiettoria_posizione(:,1))~=length(traiettoria_orientazione(:,1))
    traiettoria_posizione = resample(traiettoria_posizione, t_posizione(:,1));
    traiettoria_posizione = traiettoria_posizione(1:2:end,:);
end

toc

traiettoria_finale = [(traiettoria_posizione')/1000; traiettoria_orientazione'];
save('traiettoria_finale','traiettoria_finale')
save('t_finale','t_orientazione')
save('viapoint_finale','viapoint')
save('ostacoli','obs')
%% Plot


close all
figure
plot3(inizio(1), inizio(2), inizio(3),'ok');
hold on
grid on
% xlim([-20,80])
% ylim([-20,80])
% zlim([-20,80])
xlim([-500,0])
ylim([-900,600])
zlim([-600,900])
view(0,90)
[X,Y,Z] = sphere;
plot3(fine(1), fine(2), fine(3),'ok');
for i=1:size(obs,1)
    obsi = surf(obs(i,4)*X+obs(i,1), obs(i,4)*Y+obs(i,2), obs(i,4)*Z+obs(i,3), [1 0 0]);
    set(obsi, 'facec', 'r')
    set(obsi, 'FaceAlpha',0.1)
    set(obsi, 'edgec', [200,0,0]./255)
    plot3(obs(i,1), obs(i,2), obs(i,3), 'ob');
end

if size(viapoint,1)~=0
    for j=1:size(viapoint,1)
        plot3(viapoint(j,1), viapoint(j,2), viapoint(j,3), 'ob');
    end
end

%plot3(retta(:,1), retta(:,2), retta(:,3), '-c');
plot3(traiettoria_posizione(:,1), traiettoria_posizione(:,2), traiettoria_posizione(:,3), '-g');


figure
plot(t_posizione, traiettoria_posizione(:,1))
hold on
plot(inizio(4),inizio(1),'ko')
plot(fine(4),fine(1),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(1),'ro')
end

figure
plot(t_posizione, traiettoria_posizione(:,2),'b')
hold on
xlabel('[s]')
ylabel('[mm]')
ylim([-450 450])
grid on
plot(inizio(4),inizio(2),'ko')
plot(fine(4),fine(2),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(2),'ro')
end

figure
plot(t_posizione, traiettoria_posizione(:,3))
hold on
plot(inizio(4),inizio(3),'ko')
plot(fine(4),fine(3),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(3),'ro')
end

single_velocity = diff(traiettoria_posizione(:,2))/(t_posizione(2)-t_posizione(1));

figure
plot(t(1:end-1),single_velocity,'b')
xlabel('[s]')
ylabel('[mm/s]')
grid on

%% Calcolo Jerk

dt = 5/length(traiettoria_posizione(:,1));

velocita = [];

for i=1:3
    vel_aux = diff(traiettoria_posizione(:,i))/(dt*1000);
    velocita = [velocita vel_aux];
end

for i=1:length(velocita(:,1))
    mod_vel(i) = norm(velocita(i,:));
end

accelerazione = [];

for i=1:3
    acc_aux = diff(velocita(:,i))/dt;
    accelerazione = [accelerazione acc_aux];
end

for i=1:length(accelerazione(:,1))
    mod_acc(i) = norm(accelerazione(i,:));
end

jerk = [];

for i=1:3
    jrk_aux = diff(accelerazione(:,i))/dt;
    jerk = [jerk jrk_aux];
end

mod_jrk = [];

for i=1:length(jerk(:,1))
    mod_jrk(i) = norm(jerk(i,:));
end

avg_jerk = mean(mod_jrk)

figure
subplot(3,1,1)
plot(traiettoria_posizione(:,1))
title('posizione')
subplot(3,1,2)
plot(traiettoria_posizione(:,2))
subplot(3,1,3)
plot(traiettoria_posizione(:,3))

figure
subplot(3,1,1)
plot(velocita(:,1))
title('velocità')
subplot(3,1,2)
plot(velocita(:,2))
subplot(3,1,3)
plot(velocita(:,3))

figure
subplot(3,1,1)
plot(accelerazione(:,1))
title('accelerazione')
subplot(3,1,2)
plot(accelerazione(:,2))
subplot(3,1,3)
plot(accelerazione(:,3))

figure
subplot(3,1,1)
plot(jerk(:,1))
title('jerk')
subplot(3,1,2)
plot(jerk(:,2))
subplot(3,1,3)
plot(jerk(:,3))

figure
plot(mod_vel)

figure
plot(mod_acc)

figure
plot(mod_jrk)

%% Video Plot (RIMASTO DA COPIA-INCOLLA)
% dt = dt_target;
% 
% figure
% plot3(inizio(1), inizio(2), inizio(3),'ok');
% hold on
% grid on
% xlim([-500,0])
% ylim([-900,600])
% zlim([-600,900])
% xlabel('[mm]')
% ylabel('[mm]')
% zlabel('[mm]')
% %view(-45,45)
% view(0,90)
% plot3(traiettoria_target(1,1), traiettoria_target(1,2), traiettoria_target(1,3), 'ok');
% % for i=1:size(obs,1)
% %     obsi = surf(obs(i,4)*X+obs(i,1), obs(i,4)*Y+obs(i,2), obs(i,4)*Z+obs(i,3), [1 0 0]);
% %     set(obsi, 'facec', 'r')
% %     set(obsi, 'FaceAlpha',0.1)
% %     set(obsi, 'edgec', [200,0,0]./255)
% %     plot3(obs(i,1), obs(i,2), obs(i,3), 'ob');
% % end
% curve = animatedline('LineWidth',2);
% curve_target = animatedline('LineWidth',2);
% %title('Traiettoria fPCA')
% 
% for i=1:length(traiettoria_posizione(:,1))
%     addpoints(curve,traiettoria_posizione(i,1),traiettoria_posizione(i,2),traiettoria_posizione(i,3));
%     head = scatter3(traiettoria_posizione(i,1),traiettoria_posizione(i,2),traiettoria_posizione(i,3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
%     addpoints(curve_target,traiettoria_target(i,1),traiettoria_target(i,2),traiettoria_target(i,3));
%     head_target = scatter3(traiettoria_target(i,1),traiettoria_target(i,2),traiettoria_target(i,3),'filled','MarkerFaceColor','r','MarkerEdgeColor','r');
%     drawnow
%     F(i) = getframe(gcf);
%     pause(dt);
%     delete(head)
%     delete(head_target)
% end
% 
% video = VideoWriter('fPCA target mobile');
% video.FrameRate = round(length(traiettoria_posizione(:,1))/t_target(end));
% open(video)
% writeVideo(video,F)
% close(video)
