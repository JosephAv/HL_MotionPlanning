%% upload dati and compute of R and q

Dati_table = readtable("Dati_v_costante2.csv");
Dati = table2array(Dati_table);

Px = Dati(100:400,1);
Py = Dati(100:400,2);
Pz = Dati(100:400,3);
Vx = Dati(100:400,4);
Vy = Dati(100:400,5);
Vz = Dati(100:400,6);
Delta_T = Dati(100:400,7);

Vz1 = Vx;
Vx1 = -Vy;
Vy1 = -Vz;
Vx = Vx1;
Vy = Vy1;
Vz = Vz1;

sigma_vx = var(Vx,1);
sigma_vy = var(Vy,1);
sigma_vz = var(Vz,1);
sigma_dT = var(Delta_T,1);

dT_m = mean(Delta_T);

mvx = mean(Vx);
mvy = mean(Vy);
mvz = mean(Vz);

px0 = mean(Dati(50:75,1));
py0 = mean(Dati(50:75,2));
pz0 = mean(Dati(50:75,3));

px0 = Px(1);
py0 = Py(1);
pz0 = Pz(1);
[N,~] = size(Vx);
% Pxm
signal = zeros(N,1);
signal_value = px0;
for i=1:1:N
    signal(i,:) = signal_value;
    signal_value = signal_value + mvx*dT_m;
end
Pxm = signal;

%Pym
signal = zeros(N,1);
signal_value = py0;
for i=1:1:N
    signal(i,:) = signal_value;
    signal_value = signal_value + mvy*dT_m;
end
Pym = signal;


%Pzm
signal = zeros(N,1);
signal_value = pz0;
for i=1:1:N
    signal(i,:) = signal_value;
    signal_value = signal_value + mvz*dT_m;
end
Pzm = signal;

%% plot
% if exist('figure2') == 0  
%     figure()
% else
%     figure2()
% end
% hold on
% grid
% plot(Px,'b')
% plot(Pxm,'r')
% title('position x real vs simulated')
% legend('real','simulated')
% hold off
% 
% if exist('figure2') == 0  
%     figure()
% else
%     figure2()
% end
% hold on
% grid
% plot(Py,'b')
% plot(Pym,'r')
% title('position y real vs simulated')
% legend('real','simulated')
% hold off
% 
% if exist('figure2') == 0  
%     figure()
% else
%     figure2()
% end
% hold on
% grid
% plot(Pz,'b')
% plot(Pzm,'r')
% title('position z real vs simulated')
% legend('real','simulated')
% hold off

%% variance

sigma_px = var(Px-Pxm);
sigma_py = var(Py-Pym);
sigma_pz = var(Pz-Pzm);


%%
R = diag([sigma_px,sigma_py,sigma_pz,sigma_vx,sigma_vy,sigma_vz]);
q = sigma_dT;