% Sinusoidal trajectory 2

%% init
compute_q_R_2

%% signal simulated

N = 617;
%r = -1 + (1+1)*rand(N,1);
theta = linspace(0,2*pi,N)';

%vx
vx = 0.05*sin(theta);
%vy
vy = 0.01*sin(theta);
%vz
vz = 0.05*sin(theta);

% Sx
signal = zeros(N,1);
signal_value = 0;
for i=1:1:N
    r = -1 + (1+1)*rand(1,1);
    signal(i,:) = signal_value + 2*sqrt(sigma_px)*r;
    signal_value = signal_value + vx(i)*dT_m;
end
Sx = signal;

%Sy
signal = zeros(N,1);
signal_value = 1;
for i=1:1:N
    r = -1 + (1+1)*rand(1,1);
    signal(i,:) = signal_value + 2*sqrt(sigma_py)*r;
    signal_value = signal_value + vy(i)*dT_m;
end
Sy = signal;

%Sz
signal = zeros(N,1);
signal_value = 2;
for i=1:1:N
    r = -1 + (1+1)*rand(1,1);
    signal(i,:) = signal_value + 2*sqrt(sigma_pz)*r;
    signal_value = signal_value + vz(i)*dT_m;
end
Sz = signal;

r = -1 + (1+1)*rand(N,1);
Svx = vx + 2*sqrt(sigma_vx)*r;

r = -1 + (1+1)*rand(N,1);
Svy = vy + 2*sqrt(sigma_vy)*r;

r = -1 + (1+1)*rand(N,1);
Svz = vz + 2*sqrt(sigma_vz)*r;

r = -1 + (1+1)*rand(N,1);
SdT = dT_m + 2*sqrt(sigma_dT)*r;

%% kalman filter

[Pxf,Pyf,Pzf,Vxf,Vyf,Vzf] = kalman_filter_3(Sx,Sy,Sz,Svx,Svy,Svz,SdT,R,q);

%%

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Sx,'b')
plot(Pxf,'r')
title('position x pre vs post Kalman filter')
legend('original','filtered')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Sy,'b')
plot(Pyf,'r')
title('position y pre vs post Kalman filter')
legend('original','filtered')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Sz,'b')
plot(Pzf,'r')
%ylim([0.7 0.8])
title('position z pre vs post Kalman filter')
legend('original','filtered')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Svx,'b')
plot(Vxf,'r')
title('velocity x pre vs post Kalman filter')
legend('original','filtered')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Svy,'b')
plot(Vyf,'r')
title('velocity y pre vs post Kalman filter')
legend('original','filtered')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Svz,'b')
plot(Vzf,'r')
title('velocity z pre vs post Kalman filter')
legend('original','filtered')
hold off