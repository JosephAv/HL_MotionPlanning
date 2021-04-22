close all
clear all
clc

carica_pc;

lw = 502;
up = 757;

%DOF 1
media_dof1 = media_dof1(lw:up); 
pc1_dof1 = pc1_dof1(lw:up);
pc2_dof1 = pc2_dof1(lw:up);
pc3_dof1 = pc3_dof1(lw:up);
pc4_dof1 = pc4_dof1(lw:up);
pc5_dof1 = pc5_dof1(lw:up);

media_dof1 = media_dof1(1:2:end); 
pc1_dof1 = pc1_dof1(1:2:end);
pc2_dof1 = pc2_dof1(1:2:end);
pc3_dof1 = pc3_dof1(1:2:end);
pc4_dof1 = pc4_dof1(1:2:end);
pc5_dof1 = pc5_dof1(1:2:end);

%DOF 2
media_dof2 = media_dof2(lw:up); 
pc1_dof2 = pc1_dof2(lw:up);
pc2_dof2 = pc2_dof2(lw:up);
pc3_dof2 = pc3_dof2(lw:up);
pc4_dof2 = pc4_dof2(lw:up);
pc5_dof2 = pc5_dof2(lw:up);

media_dof2 = media_dof2(1:2:end); 
pc1_dof2 = pc1_dof2(1:2:end);
pc2_dof2 = pc2_dof2(1:2:end);
pc3_dof2 = pc3_dof2(1:2:end);
pc4_dof2 = pc4_dof2(1:2:end);
pc5_dof2 = pc5_dof2(1:2:end);

%DOF 3
media_dof3 = media_dof3(lw:up); 
pc1_dof3 = pc1_dof3(lw:up);
pc2_dof3 = pc2_dof3(lw:up);
pc3_dof3 = pc3_dof3(lw:up);
pc4_dof3 = pc4_dof3(lw:up);
pc5_dof3 = pc5_dof3(lw:up);

media_dof3 = media_dof3(1:2:end); 
pc1_dof3 = pc1_dof3(1:2:end);
pc2_dof3 = pc2_dof3(1:2:end);
pc3_dof3 = pc3_dof3(1:2:end);
pc4_dof3 = pc4_dof3(1:2:end);
pc5_dof3 = pc5_dof3(1:2:end);

%DOF 4
media_dof4 = media_dof4(lw:up); 
pc1_dof4 = pc1_dof4(lw:up);

media_dof4 = media_dof4(1:2:end); 
pc1_dof4 = pc1_dof4(1:2:end);

%DOF 5
media_dof5 = media_dof5(lw:up); 
pc1_dof5 = pc1_dof5(lw:up);

media_dof5 = media_dof5(1:2:end); 
pc1_dof5 = pc1_dof5(1:2:end);

%DOF 6
media_dof6 = media_dof6(lw:up); 
pc1_dof6 = pc1_dof6(lw:up);

media_dof6 = media_dof6(1:2:end); 
pc1_dof6 = pc1_dof6(1:2:end);

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
%Definizione dei vincoli iniziali e finali sulla traiettoria. Ogni punto è
%definito come [x y z t] dove t è l'istante temporale a cui bisogna
%trovarsi nel determinato punto. L'unità di misura utilizzata non è
%rilevante a patto di essere coerenti con quella scelta (in questo caso
%sono mm per le traslazioni e rad per le rotazioni). L'orietamento viene
%definito con la convenzione RPY.

inizio = [-200 400 400 0];
fine = [-205 -400 405 5];
ori_inizio = [0 0 0 0];
ori_fine = [0 0 pi/3 5];
v_start = 0;
a_start = 0;

l = length(media_dof1);

%% Definizione Ostacoli
%In questo esempio vengono presi in considerazione ostacoli sferici.
%All'interno della matrice obs ogni riga [x y z r] rappresenta una sfera
%con raggio r e centro in (x,y,z). L'unità di misura deve essere coerente
%con quella della traiettoria

obs = [-220 -100 400 70];

%% Orientazione

traiettoria_orientazione = [];
t_orientazione = [];

for i=1:3
    [orientazione, t] = traj_no_obs(dof{i+3}, ori_inizio(i), ori_fine(i), ori_fine(4));
    traiettoria_orientazione = [traiettoria_orientazione orientazione];
    t_orientazione = [t_orientazione t];
end

%% Traiettoria Diretta
offset = 50;
tic
traiettoria_diretta = [];
t_diretta = [];

for i=1:3
    [traiettoria, t] = traj_no_obs(dof{i}, inizio(i), fine(i), ori_fine(4));
    traiettoria_diretta = [traiettoria_diretta traiettoria];
    t_diretta = [t_diretta t];
end
toc

tic
index = 50; %Distanza minima che può avere la traiettoria rispetto ad un generico ostacolo
bivio = traiettoria_diretta(index,:);
t_bivio = t_diretta(index,1);
v_bivio = (traiettoria_diretta(index,:)-traiettoria_diretta(index-1,:))/(t_diretta(index,1)-t_diretta(index-1,1));
v_bivio2 = (traiettoria_diretta(index-1,:)-traiettoria_diretta(index-2,:))/(t_diretta(index-1,1)-t_diretta(index-2,1));
a_bivio = (v_bivio-v_bivio2)/(t_diretta(index,1)-t_diretta(index-1,1));

%1)Raccordo traiettoria mantenendo i vincoli temporali
% [traiettoria_posizione, t_posizione, viapoint] = calcolo_traj_2(bivio,t_bivio,fine(1:3),fine(4),obs,offset,dof,v_bivio,a_bivio, 'r');

%2)Raccordo traiettoria aumentando il tempo a disposizione
[traiettoria_posizione, t_posizione, viapoint] = calcolo_traj_2(bivio,0,fine(1:3),fine(4),obs,offset,dof,v_bivio,a_bivio, 'r');


toc

traiettoria_modificata = [traiettoria_diretta(1:index,:); traiettoria_posizione(2:end,:)];

%1)Creazione asse dei tempi 
%t_modificata = [t_diretta(1:index,:); t_posizione(2:end,:)];

%2)Creazione asse dei tempi  
t_posizione = t_posizione+t_bivio;
t_modificata = [t_diretta(1:index,:); t_posizione(2:end,:)];
viapoint(4) = viapoint(4)+t_bivio;


traiettoria_modificata = resample(traiettoria_modificata, t_modificata(:,1));
t_modificata = resample(t_modificata, t_modificata(:,1));

% traiettoria_modificata = traiettoria_modificata(1:5:end,:);
% t_modificata = t_modificata(1:5:end,:);

traiettoria_finale_r = [traiettoria_modificata, zeros(length(traiettoria_modificata(:,1)),3)];
t_finale_r = [t_modificata, t_modificata];

viapoint_finale_r = viapoint;


save('traiettoria_finale_r','traiettoria_finale_r')
save('t_finale_r','t_finale_r')
save('ostacoli','obs')
save('viapoint_finale_r','viapoint_finale_r')

figure
plot(t_modificata(:,1),traiettoria_modificata(:,1),'*')

distanza = [];
for i = 1: length(traiettoria_modificata(:,1))
    distanza(i) = norm(traiettoria_modificata(i,:)-obs(1:3))-obs(4);
end

%% Plot


figure
plot3(inizio(1), inizio(2), inizio(3),'ok');
hold on
grid on
% xlim([-20,80])
% ylim([-20,80])
% zlim([-20,80])
xlim([-600 600])
ylim([-600 600])
zlim([-600 600])
view(0,90)
plot3(fine(1), fine(2), fine(3), 'ok');
[X,Y,Z] = sphere;
   
for i=1:size(obs,1)
    obsi = surf(obs(i,4)*X+obs(i,1), obs(i,4)*Y+obs(i,2), obs(i,4)*Z+obs(i,3), [1 0 0]);
    set(obsi, 'facec', 'r')
    set(obsi, 'FaceAlpha',0.1)
    set(obsi, 'edgec', [200,0,0]./255)
    plot3(obs(i,1), obs(i,2), obs(i,3), 'ob');
end

for j=1:size(viapoint,1)
    plot3(viapoint(j,1), viapoint(j,2), viapoint(j,3), 'ob');
end

%plot3(retta(:,1), retta(:,2), retta(:,3), '-c');
plot3(traiettoria_posizione(:,1), traiettoria_posizione(:,2), traiettoria_posizione(:,3), '-g');
plot3(traiettoria_diretta(:,1), traiettoria_diretta(:,2), traiettoria_diretta(:,3), '-');

figure
plot(t_diretta(:,1), traiettoria_diretta(:,1))
hold on
plot(t_posizione(:,1), traiettoria_posizione(:,1),'g')
plot(inizio(4),inizio(1),'ko')
plot(fine(4),fine(1),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(1),'ro')
end

figure
plot(t_diretta(:,1), traiettoria_diretta(:,2))
hold on
plot(t_posizione(:,1), traiettoria_posizione(:,2),'g')
plot(inizio(4),inizio(2),'ko')
plot(fine(4),fine(2),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(2),'ro')
end

figure
plot(t_diretta(:,1), traiettoria_diretta(:,3))
hold on
plot(t_posizione(:,1), traiettoria_posizione(:,3),'g')
plot(inizio(4),inizio(3),'ko')
plot(fine(4),fine(3),'ko')
if size(viapoint,1)~=0
    plot(viapoint(4),viapoint(3),'ro')
end

figure
plot(t_orientazione(:,1), traiettoria_orientazione(:,1))

figure
plot(t_orientazione(:,1), traiettoria_orientazione(:,2))

figure
plot(t_orientazione(:,1), traiettoria_orientazione(:,3))

figure
plot(t_modificata, distanza)

%% Video Plot
% dt = t_modificata(2,1)-t_modificata(1,1);
% 
% figure
% plot3(inizio(1), inizio(2), inizio(3),'ok');
% hold on
% grid on
% xlim([-600 600])
% ylim([-600 600])
% zlim([0 600])
% xlabel('[mm]')
% ylabel('[mm]')
% zlabel('[mm]')
% view(-45,45)
% plot3(traiettoria_diretta(:,1), traiettoria_diretta(:,2), traiettoria_diretta(:,3), '-');
% plot3(fine(1), fine(2), fine(3), 'ok');
% curve = animatedline('LineWidth',2);
% %title('Traiettoria fPCA')
% 
% index_cor = index+25;
% 
% for i=1:length(traiettoria_modificata(:,1))
%     addpoints(curve,traiettoria_modificata(i,1),traiettoria_modificata(i,2),traiettoria_modificata(i,3));
%     head = scatter3(traiettoria_modificata(i,1),traiettoria_modificata(i,2),traiettoria_modificata(i,3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b');
%     drawnow
%     if i==index_cor
%         for i=1:size(obs,1)
%             obsi = surf(obs(i,4)*X+obs(i,1), obs(i,4)*Y+obs(i,2), obs(i,4)*Z+obs(i,3), [1 0 0]);
%             set(obsi, 'facec', 'r')
%             set(obsi, 'FaceAlpha',0.1)
%             set(obsi, 'edgec', [200,0,0]./255)
%             obstacle = plot3(obs(i,1), obs(i,2), obs(i,3), 'ob');
%         end
%     end
%     
%     F(i) = getframe(gcf);
%     pause(dt);
%     delete(head)
% end
% 
% F(index_cor) = [];
% 
% video = VideoWriter('fPCA replanning');
% video.FrameRate = round(length(traiettoria_modificata(:,1))/t_modificata(end,1));
% open(video)
% writeVideo(video,F)
% close(video)