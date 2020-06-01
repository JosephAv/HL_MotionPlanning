%% EKF AVAMBRACCIO
%Prende in ingresso i dati temporali dei marker e con questi stima
%posizione e orientamento rispetto al sistema di riferimento sul torace 
%del corpo a cui sono attaccati
clear all
clc

load('Dataset_Avambraccio')

%% Posizioni locali dei marker

l_lungo = 55;
l_corto = 45;

% Posizione fisica marker 
% 2  1
% 4  3
% 6  5

%Sistema di riferimento braccio: asse y positivo uscente dalla base
%                                asse z positivo in direzione distale 
%                                asse x di conseguenza

ArmLocalCoords = [+l_corto/2         0         -l_lungo;...  %MK1
                  -l_corto/2         0         -l_lungo;...  %MK2
                  +l_corto/2         0          0;...        %MK3
                  -l_corto/2         0          0;...        %MK4
                  +l_corto/2         0         +l_lungo;...  %MK5
                  -l_corto/2         0         +l_lungo];    %MK6

%% Filtro di Kalman esteso

tic
Q = 1*blkdiag(1*eye(3),0.01*eye(3));   % incertezza sulla predizione
R = 100*eye(18);                      % incertezza sulle misure
Dataset_Posestimate = [];

for j = 1:length(Dataset_Avambraccio)

    movim = Dataset_Avambraccio{j};
    posamov = [];
    stima = zeros(6,1);
    P = 0.001*blkdiag(1*eye(3),1*eye(3));   % incertezza sulla stima
    for i = 1:length(movim)  %length(data)=numero di istanti temporali del movimento
        K1 = 1;
        K2 = 1;
        K3 = 1;
        K4 = 1;
        K5 = 1;
        K6 = 1;
        %stima=stima   %Predizione dello stato x=x_old
        P = P + Q;
        H = [calcolo_H(stima, ArmLocalCoords(1,:)); ...
            calcolo_H(stima, ArmLocalCoords(2,:)); ...
            calcolo_H(stima, ArmLocalCoords(3,:)); ...
            calcolo_H(stima, ArmLocalCoords(4,:)); ...
            calcolo_H(stima, ArmLocalCoords(5,:)); ...
            calcolo_H(stima, ArmLocalCoords(6,:))];
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
        if mov(13:15) == [0;0;0]
            K5 = 0;
        end
        if mov(16:18) == [0;0;0]
            K6 = 0;
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
         
        SEL = blkdiag(K1*eye(3), K2*eye(3), K3*eye(3), K4*eye(3), K5*eye(3), K6*eye(3));
        h_avambraccio = [h_fun(stima,ArmLocalCoords(1,:)); ...
                  h_fun(stima,ArmLocalCoords(2,:)); ...
                  h_fun(stima,ArmLocalCoords(3,:)); ...
                  h_fun(stima,ArmLocalCoords(4,:)); ...
                  h_fun(stima,ArmLocalCoords(5,:)); ...
                  h_fun(stima,ArmLocalCoords(6,:))];
        correz = SEL*(movim{i}-h_avambraccio);
        stima = stima + K*correz;
        P = P - K*SEL*H*P;
        posamov(:,i) = stima;
    end
    Dataset_Posestimate{j,1} = posamov;
end
toc

tic
save('Dataset_Posestimate_Avambraccio', 'Dataset_Posestimate')
toc

%% VISUALIZZA LE STIME DI POSIZIONE E ORIENTAZIONE

% m = 1; %movimento scelto
% Posa = Dataset_Posestimate_Braccio{m};
% 
% figure(2)
% for i = 1:6
%     subplot(6,1,i)
%     plot(Posa(i,:))
% end