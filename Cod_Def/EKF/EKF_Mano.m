%% EKF MANO
%Prende in ingresso i dati temporali dei marker e con questi stima
%posizione e orientamento rispetto al sistema di riferimento sul torace 
%del corpo a cui sono attaccati
clear all
clc

load('Dataset_Mano')

%% Posizioni locali dei marker

l_lungo = 31;
l_corto = 14;
%diagon = sqrt(l_lungo^2+l_corto^2);

%HandIDS(1) prossimale lato pollice
%HandIDS(2) prossimale lato mignolo
%HandIDS(3) distale lato pollice
%HandIDS(4) distale lato mignolo

%Sistema di riferimento mano: asse x direzione pollice
%                             asse y uscente dal dorso
%                             asse z direzione dita

HandLocalCoords = [ l_lungo/2     0            -l_corto/2;... %MK1
                   -l_lungo/2     0            -l_corto/2;... %MK2
                    l_lungo/2     0             l_corto/2;... %MK3
                   -l_lungo/2     0             l_corto/2];   %MK4

%% Filtro di Kalman esteso

tic
Q = 1*blkdiag(1*eye(3),0.01*eye(3));   % incertezza sulla predizione
R = 100*eye(12);                      % incertezza sulle misure
Dataset_Posestimate = [];

for j = 1:length(Dataset_Mano)
    movim = Dataset_Mano{j};
    posamov = [];
    stima = zeros(6,1);
    P = 0.001*blkdiag(1*eye(3),1*eye(3));   % incertezza sulla stima
    for i = 1:length(movim)  %length(data)=numero di istanti temporali del movimento
        K1 = 1;
        K2 = 1;
        K3 = 1;
        K4 = 1;
        %stima=stima   %Predizione dello stato x=x_old
        P = P + Q;
        H = [calcolo_H(stima, HandLocalCoords(1,:)); calcolo_H(stima, HandLocalCoords(2,:)); calcolo_H(stima, HandLocalCoords(3,:)); calcolo_H(stima, HandLocalCoords(4,:))];
        S = H*P*H' + R;
        K = P*H'*inv(S);
        mov = movim{i};
        
        %Controllo per escludere marker piazzati nell'origine
        %Questo perchè se un marker non viene visto in fase di acquisizione viene messo 
        %di default all'origine
        if mov(1:3) == [0;0;0]
            K1 = 0;
        end
        if mov(4:6) == [0;0;0]
            K2 = 0;
        end
        if mov(7:9) == [0;0;0]
            K3 = 0;
        end
        if mov(10:12) == [0;0;0]
            K4 = 0;
        end
        
%         scala = 15;
%         trasl = diagon + 30; 
%         
%         if i > 50
%             K1 = K1*(1 - exp((norm(mov(1:3)-stima(1:3))-trasl)/scala)/(1+exp((norm(mov(1:3)-stima(1:3))-trasl)/scala)));
%             K2 = K2*(1 - exp((norm(mov(4:6)-stima(1:3))-trasl)/scala)/(1+exp((norm(mov(4:6)-stima(1:3))-trasl)/scala)));
%             K3 = K3*(1 - exp((norm(mov(7:9)-stima(1:3))-trasl)/scala)/(1+exp((norm(mov(7:9)-stima(1:3))-trasl)/scala)));
%             K4 = K4*(1 - exp((norm(mov(10:12)-stima(1:3))-trasl)/scala)/(1+exp((norm(mov(10:12)-stima(1:3))-trasl)/scala)));
%         end
         
        SEL = blkdiag(K1*eye(3), K2*eye(3), K3*eye(3), K4*eye(3));
        h_mano = [h_fun(stima,HandLocalCoords(1,:)); ...
                  h_fun(stima,HandLocalCoords(2,:)); ...
                  h_fun(stima,HandLocalCoords(3,:)); ...
                  h_fun(stima,HandLocalCoords(4,:))];
        correz = SEL*(movim{i}-h_mano);
        stima = stima + K*correz;
        P = P - K*SEL*H*P;
        posamov(:,i) = stima;
    end
    Dataset_Posestimate{j,1} = posamov;
end
toc

tic
save('Dataset_Posestimate_Mano', 'Dataset_Posestimate')
toc

%% VISUALIZZA LE STIME DI POSIZIONE E ORIENTAZIONE

% m = 1; %movimento scelto
% Posa = Dataset_Posestimate{m};
% 
% figure(2)
% for i = 1:6
%     subplot(6,1,i)
%     plot(Posa(i,:))
% end