function [traiettoria, t] = traj_obs(fpc,start,finish,v_start, a_start, t_f, vp)

%Utilizzo dell'intera fPC fra due viapoint vincolando fino
%all'accelerazione nei raccordi

%% Definizione velocità e accelerazioni iniziali e finali
media = fpc(:,1);
pc1 = fpc(:,2);
pc2 = fpc(:,3);
pc3 = fpc(:,4);
pc4 = fpc(:,5);
pc5 = fpc(:,6);
l = length(media);

[n, ~] = size(vp);

trg = vp(1,1);
time = vp(1,2);
t_c = time/(l-1);

vm_s = (media(2) - media(1))/t_c;
vm_s1 = (media(3) - media(2))/t_c;
am_s = (vm_s1 - vm_s)/t_c;
vm_f = (media(l) - media(l-1))/t_c;
vm_f1 = (media(l-1) - media(l-2))/t_c;
am_f = (vm_f - vm_f1)/t_c;

v1_s = (pc1(2) - pc1(1))/t_c;
v1_s1 = (pc1(3) - pc1(2))/t_c;
a1_s = (v1_s1 - v1_s)/t_c;
v1_f = (pc1(l) - pc1(l-1))/t_c;
v1_f1 = (pc1(l-1) - pc1(l-2))/t_c;
a1_f = (v1_f - v1_f1)/t_c;

v2_s = (pc2(2) - pc2(1))/t_c;
v2_s1 = (pc2(3) - pc2(2))/t_c;
a2_s = (v2_s1 - v2_s)/t_c;
v2_f = (pc2(l) - pc2(l-1))/t_c;
v2_f1 = (pc2(l-1) - pc2(l-2))/t_c;
a2_f = (v2_f - v2_f1)/t_c;

v3_s = (pc3(2) - pc3(1))/t_c;
v3_s1 = (pc3(3) - pc3(2))/t_c;
a3_s = (v3_s1 - v3_s)/t_c;
v3_f = (pc3(l) - pc3(l-1))/t_c;
v3_f1 = (pc3(l-1) - pc3(l-2))/t_c;
a3_f = (v3_f - v3_f1)/t_c;

v4_s = (pc4(2) - pc4(1))/t_c;
v4_s1 = (pc4(3) - pc4(2))/t_c;
a4_s = (v4_s1 - v4_s)/t_c;
v4_f = (pc4(l) - pc4(l-1))/t_c;
v4_f1 = (pc4(l-1) - pc4(l-2))/t_c;
a4_f = (v4_f - v4_f1)/t_c;

v5_s = (pc5(2) - pc5(1))/t_c;
v5_s1 = (pc5(3) - pc5(2))/t_c;
a5_s = (v5_s1 - v5_s)/t_c;
v5_f = (pc5(l) - pc5(l-1))/t_c;
v5_f1 = (pc5(l-1) - pc5(l-2))/t_c;
a5_f = (v5_f - v5_f1)/t_c;


if n<2
    v_f = (finish - trg)/(t_f - time);
%     v_f = v_f/2;
%     a_f = -v_f/(t_f - time);
    a_f = 0;
elseif n==2
    v_f = (vp(2,1) - trg)/(vp(2,2) - time);
%     v_f = v_f/2;
    v_f1 = (finish - vp(2,1))/(t_f - vp(2,2));
%     v_f1 = (v_f1+v_f)/2;
    a_f = (v_f1 - v_f)/(t_f - vp(2,2));
else
    v_f = (vp(2,1) - trg)/(vp(2,2)-time);
%     v_f = v_f/2;
    v_f1 = (vp(3,1) - vp(2,1))/(vp(3,2)-vp(2,2));
%     v_f1 = (v_f1+v_f)/2;
    a_f = (v_f1 - v_f)/(vp(3,2)-vp(2,2));
end

%% Versione Originale

% v_f = (finish-start)/t_f;
% a_f = 0;
% 
% b = [start-media(1); trg-media(l); (v_start-vm_s); (v_f-vm_f); (a_start-am_s); (a_f-am_f)];
%     
% A = [1           pc1(1)        pc2(1)        pc3(1)         pc4(1)         pc5(1);...
%      1           pc1(l)        pc2(l)        pc3(l)         pc4(l)         pc5(l);...
%      0             v1_s          v2_s          v3_s           v4_s           v5_s;...
%      0             v1_f          v2_f          v3_f           v4_f           v5_f;...
%      0             a1_s          a2_s          a3_s           a4_s           a5_s;...
%      0             a1_f          a2_f          a3_f           a4_f           a5_f];
% A = inv(A);
% x = A*b;
% 
% X_tot = [x];
% 
% traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3 + x(5).*pc4 + x(6).*pc5;
% 
% traiettoria = traj;

%% Versione senza vincoli a sx del viapoint

% b = [start-media(1); trg-media(l); (v_start-vm_s); (a_start-am_s)];
%     
% A = [1           pc1(1)        pc2(1)        pc3(1);...
%      1           pc1(l)        pc2(l)        pc3(l);...
%      0             v1_s          v2_s          v3_s;...
%      0             a1_s          a2_s          a3_s];
% A = inv(A);
% x = A*b;
% 
% X_tot = [x; 0; 0];
% 
% traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3;
% 
% traiettoria = traj;

%% Versione senza continuità di accelerazione in partenza

% b = [start-media(1); trg-media(l); (v_start-vm_s)];
%     
% A = [1           pc1(1)        pc2(1);...
%      1           pc1(l)        pc2(l);...
%      0             v1_s          v2_s];
% A = inv(A);
% x = A*b;
% 
% X_tot = [x; 0; 0; 0];
% 
% traj = x(1) + media + x(2).*pc1 + x(3).*pc2;
% 
% traiettoria = traj;

%% Versione senza continuità di accelerazione vincolando la velocità in arrivo [VERSIONE MIGLIORE]

% v_f = (finish-start)/t_f;

v_f = (trg-start)/(time);
% v_f = (v_f+v_1)/2;


b = [start-media(1); trg-media(l); (v_start-vm_s); (v_f-vm_f)];
    
A = [1           pc1(1)        pc2(1)        pc3(1);...
     1           pc1(l)        pc2(l)        pc3(l);...
     0             v1_s          v2_s          v3_s;...
     0             v1_f          v2_f          v3_f];...

A = inv(A);
x = A*b;

X_tot = [x; 0; 0];

traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3;

traiettoria = traj;

%% Versione con continuità completa e vincolo di velocità in arrivo

% v_f = (finish-start)/t_f;
% 
% b = [start-media(1); trg-media(l); (v_start-vm_s); (v_f-vm_f); (a_start-am_s)];
%     
% A = [1           pc1(1)        pc2(1)        pc3(1)         pc4(1);...
%      1           pc1(l)        pc2(l)        pc3(l)         pc4(l);...
%      0             v1_s          v2_s          v3_s           v4_s;...
%      0             v1_f          v2_f          v3_f           v4_f;...
%      0             a1_s          a2_s          a3_s           a4_s];...
% 
% A = inv(A);
% x = A*b;
% 
% X_tot = [x; 0];
% 
% traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3 + x(5).*pc4;
% 
% traiettoria = traj;

%% Versione solo continuità di posizione
% 
% b = [start-media(1); trg-media(l)];
%     
% A = [1           pc1(1);...
%      1           pc1(l)];
% 
% 
% A = inv(A);
% x = A*b;
% 
% X_tot = [x; 0; 0; 0; 0];
% 
% traj = x(1) + media + x(2).*pc1;
% 
% traiettoria = traj;

%% Solo continuità di posizione e velocità viapoint
 
% v_f = (finish-start)/t_f;
% 
%  b = [start-media(1); trg-media(l); v_f-vm_f];
%     
%  A = [1    pc1(1)    pc2(1);...
%       1    pc1(l)    pc2(l);...
%       0      v1_f      v2_f];
%     
%  A = inv(A);
%  x = A*b;
%  
%  X_tot = [x;0;0;0];
%     
%  traj = x(1) + media + x(2).*pc1 + x(3).*pc2;
%  
%  traiettoria = traj;

%%

t = (t_c*[0:l-1])';

v_old = (traiettoria(end)-traiettoria(end-1))/(t(end)-t(end-1));
v_old1 = (traiettoria(end-1)-traiettoria(end-2))/(t(end)-t(end-1));
a_old = (v_old-v_old1)/(t(end)-t(end-1));




% 

t_c = (t_f-time)/(l-1);

vm_s = (media(2) - media(1))/t_c;
vm_s1 = (media(3) - media(2))/t_c;
am_s = (vm_s1 - vm_s)/t_c;
vm_f = (media(l) - media(l-1))/t_c;
vm_f1 = (media(l-1) - media(l-2))/t_c;
am_f = (vm_f - vm_f1)/t_c;

v1_s = (pc1(2) - pc1(1))/t_c;
v1_s1 = (pc1(3) - pc1(2))/t_c;
a1_s = (v1_s1 - v1_s)/t_c;
v1_f = (pc1(l) - pc1(l-1))/t_c;
v1_f1 = (pc1(l-1) - pc1(l-2))/t_c;
a1_f = (v1_f - v1_f1)/t_c;

v2_s = (pc2(2) - pc2(1))/t_c;
v2_s1 = (pc2(3) - pc2(2))/t_c;
a2_s = (v2_s1 - v2_s)/t_c;
v2_f = (pc2(l) - pc2(l-1))/t_c;
v2_f1 = (pc2(l-1) - pc2(l-2))/t_c;
a2_f = (v2_f - v2_f1)/t_c;

v3_s = (pc3(2) - pc3(1))/t_c;
v3_s1 = (pc3(3) - pc3(2))/t_c;
a3_s = (v3_s1 - v3_s)/t_c;
v3_f = (pc3(l) - pc3(l-1))/t_c;
v3_f1 = (pc3(l-1) - pc3(l-2))/t_c;
a3_f = (v3_f - v3_f1)/t_c;

v4_s = (pc4(2) - pc4(1))/t_c;
v4_s1 = (pc4(3) - pc4(2))/t_c;
a4_s = (v4_s1 - v4_s)/t_c;
v4_f = (pc4(l) - pc4(l-1))/t_c;
v4_f1 = (pc4(l-1) - pc4(l-2))/t_c;
a4_f = (v4_f - v4_f1)/t_c;

v5_s = (pc5(2) - pc5(1))/t_c;
v5_s1 = (pc5(3) - pc5(2))/t_c;
a5_s = (v5_s1 - v5_s)/t_c;
v5_f = (pc5(l) - pc5(l-1))/t_c;
v5_f1 = (pc5(l-1) - pc5(l-2))/t_c;
a5_f = (v5_f - v5_f1)/t_c;

 %% Versione Originale
 
%  b = [traj(end)-media(1); finish-media(l); v_old-vm_s; -vm_f; a_old-am_s];
%     
%  A = [1    pc1(1)    pc2(1)   pc3(1)    pc4(1);...
%       1    pc1(l)    pc2(l)   pc3(l)    pc4(l);...
%       0      v1_s      v2_s     v3_s      v4_s;...
%       0      v1_f      v2_f     v3_f      v4_f;...
%       0      a1_s      a2_s     a3_s      a4_s];
%     
%  A = inv(A);
%  x = A*b;
%  
%  X_tot = [X_tot [x;0]];
%     
%  traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3 + x(5).*pc4;
%  t_traj = (t_c*[0:(l-1)])' + t(end);
%     
%  traiettoria = [traiettoria; traj(2:end)];
%  t = [t; t_traj(2:end)];
 
 %% Senza continuità di accelerazione [VERSIONE MIGLIORE]
 
 b = [traj(end)-media(1); finish-media(l); v_old-vm_s; -vm_f];
    
 A = [1    pc1(1)    pc2(1)   pc3(1);...
      1    pc1(l)    pc2(l)   pc3(l);...
      0      v1_s      v2_s     v3_s;...
      0      v1_f      v2_f     v3_f];
    
 A = inv(A);
 x = A*b;
 
 X_tot = [X_tot [x;0;0]];
    
 traj = x(1) + media + x(2).*pc1 + x(3).*pc2 + x(4).*pc3;
 t_traj = (t_c*[0:(l-1)])' + t(end);
    
 traiettoria = [traiettoria; traj(2:end)];
 t = [t; t_traj(2:end)];

 
%% Solo continuità di posizione e velocità finale
 
%  b = [traj(end)-media(1); finish-media(l); -vm_f];
%     
%  A = [1    pc1(1)    pc2(1);...
%       1    pc1(l)    pc2(l);...
%       0      v1_f      v2_f];
%     
%  A = inv(A);
%  x = A*b;
%  
%  X_tot = [X_tot [x;0;0;0]];
%     
%  traj = x(1) + media + x(2).*pc1 + x(3).*pc2;
%  t_traj = (t_c*[0:(l-1)])' + t(end);
%     
%  traiettoria = [traiettoria; traj(2:end)];
%  t = [t; t_traj(2:end)];
end