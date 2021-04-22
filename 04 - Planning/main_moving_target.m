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

%% Target Mobile
%Definizione movimento del target. A titolo di esempio è stato preso un
%moto rettilineo uniforme da intercettare a metà della sua traiettoria

start_target = [-205 -400 405];
end_target = 2*start_target;

t_tot = 5;
t_intercetto = 2.5;
dt_target = t_intercetto/255;

t_target = 0:dt_target:t_tot;

traiettoria_target = (end_target-start_target).*t_target'/t_tot + start_target;

v_target = (traiettoria_target(2,:)-traiettoria_target(1,:))/(t_target(2)-t_target(1));

index = 256;


%% Definizione Inizio e Fine
%Definizione dei vincoli iniziali e finali sulla traiettoria. Ogni punto è
%definito come [x y z t] dove t è l'istante temporale a cui bisogna
%trovarsi nel determinato punto. L'unità di misura utilizzata non è
%rilevante a patto di essere coerenti con quella scelta (in questo caso
%sono mm per le traslazioni e rad per le rotazioni). L'orietamento viene
%definito con la convenzione RPY.

inizio = [-200 400 400 0];
fine = [-200 -400 400 5];
ori_inizio = [0 0 -pi/2 0];
ori_fine = [0 0 pi/2 5];
v_start = 0;
v_finish = 0;

l = length(media_dof1);

%% Definizione Ostacoli
%ANCORA DA IMPLEMENTARE L'EVITAMENTO OSTACOLI CON VELOCITà FINALE NON NULLA
%La struttura di definizione ostacoli e ricalcolo della traiettoria è stata
%mantenuta nel codice per sviluppi futuri

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
    [traiettoria, t] = traj_no_obs_new(dof{i}, inizio(i), traiettoria_target(index,i), t_target(index), v_start, v_target(i));
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
    [traiettoria, t] = traj_no_obs(dof{i}, ori_inizio(i-3), ori_fine(i-3), t_target(index));
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
d = length(traiettoria_posizione(:,1));

traiettoria_posizione = [traiettoria_posizione; traiettoria_target(d+1:end,:)];

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
plot3(traiettoria_target(1,1), traiettoria_target(1,2), traiettoria_target(1,3), 'ok');
[X,Y,Z] = sphere;
   
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
plot3(traiettoria_target(:,1), traiettoria_target(:,2), traiettoria_target(:,3), '-r');

figure
plot(t_target, traiettoria_posizione(:,1))
hold on
plot(inizio(4),inizio(1),'ko')
%plot(fine(4),fine(1),'ko')
plot(t_target, traiettoria_target(:,1),'-r')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(1),'ro')
end

figure
plot(t_target, traiettoria_posizione(:,2))
hold on
plot(inizio(4),inizio(2),'ko')
%plot(fine(4),fine(2),'ko')
plot(t_target, traiettoria_target(:,2),'-r')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(2),'ro')
end

figure
plot(t_target, traiettoria_posizione(:,3))
hold on
plot(inizio(4),inizio(3),'ko')
%plot(fine(4),fine(3),'ko')
plot(t_target, traiettoria_target(:,3),'-r')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(3),'ro')
end

%% Video Plot
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
