% i segnali di interesse sono x e z, y è quello dell'altezza da terra
% dell'oggetto( che dovrebbe rimanere costante ). La prova è stata compiuta
% con l'oggetto che si muove con v costante in direzione obligua e verso la
% camera

compute_q_R_2
%%
Dati_table = readtable("Dati_v_costante3.csv");
Dati = table2array(Dati_table);

Px = Dati(:,1);
Py = Dati(:,2);
Pz = Dati(:,3);
Vx = Dati(:,4);
Vy = Dati(:,5);
Vz = Dati(:,6);
Delta_T = Dati(:,7);

Vz1 = Vx;
Vx1 = -Vy;
Vy1 = -Vz;
Vx = Vx1;
Vy = Vy1;
Vz = Vz1;

%% plot

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Px,'b')
title('position x')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Py,'b')
title('position y')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Pz,'b')
title('position z')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vx,'b')
title('velocity x')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vy,'b')
title('velocity y')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vz,'b')
title('velocity z')
hold off

%% cut signals

Px = Px(200:400,1);
Py = Py(200:400,1);
Pz = Pz(200:400,1);
Vx = Vx(200:400,1);
Vy = Vy(200:400,1);
Vz = Vz(200:400,1);
Delta_T = Delta_T(200:400,1);



%% kalman filter

[x_filtrato(:,1),x_filtrato(:,2),x_filtrato(:,3),x_filtrato(:,4),x_filtrato(:,5),x_filtrato(:,6)] = kalman_filter_3(Px,Py,Pz,Vx,Vy,Vz,Delta_T,R,q);


%% plot

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Px,'b')
plot(x_filtrato(:,1),'r')
title('position x pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m]')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Py,'b')
plot(x_filtrato(:,2),'r')
title('position y pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m]')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Pz,'b')
plot(x_filtrato(:,3),'r')
%ylim([0.7 0.8])
title('position z pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m]')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vx,'b')
plot(x_filtrato(:,4),'r')
title('velocity x pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m/s]')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vy,'b')
plot(x_filtrato(:,5),'r')
title('velocity y pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m/s]')
hold off

if exist('figure2') == 0  
    figure()
else
    figure2()
end
hold on
grid
plot(Vz,'b')
plot(x_filtrato(:,6),'r')
title('velocity z pre vs post Kalman filter')
legend('original','filtered')
ylabel('[m/s]')
hold off