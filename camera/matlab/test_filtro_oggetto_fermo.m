compute_q_R_2

%% upload dati
Dati_table = readtable("Dati_Pos3.csv");
Dati = table2array(Dati_table);

Px = Dati(:,1);
Py = Dati(:,2);
Pz = Dati(:,3);
Vx = Dati(:,4);
Vy = Dati(:,5);
Vz = Dati(:,6);
Delta_T = Dati(:,7);

%% kalman filter

[x_filtrato(:,1),x_filtrato(:,2),x_filtrato(:,3),x_filtrato(:,4),x_filtrato(:,5),x_filtrato(:,6)] = kalman_filter_3(Px,Py,Pz,Vx,Vy,Vz,Delta_T,R,q);

%%
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
plot(Vx,'b')
plot(x_filtrato(:,4),'r')
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
plot(Vy,'b')
plot(x_filtrato(:,5),'r')
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
plot(Vz,'b')
plot(x_filtrato(:,6),'r')
title('velocity z pre vs post Kalman filter')
legend('original','filtered')
hold off


