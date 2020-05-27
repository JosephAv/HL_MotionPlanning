close all
clear all
clc

%% Estrazione dalle cartelle di DATA_to_share
lista_soggetti
Ntask = 30;

%Nsubj = 6;  %Prova codice
Nsubj = 33;
Maxnumrep = 6;
Dataset_Estratto = cell(Nsubj*Ntask*Maxnumrep,1);

tic
for i=1:Nsubj
    pathname = ['DATA_to_share/' Subj{i}];
    addpath(pathname);
    IDS_Definition;
    ID_Marker = MkIDS_UL;
    
    for j=1:Ntask
        for k=1:Maxnumrep
            try
                filename = [Subj{i} '_' num2str(j) '_' num2str(k) '.mat'];
                tmp = load(filename);
                tmp = tmp.data;
                for t=1:length(tmp)
                    marker_estratti = zeros(20,4);
                    frame = tmp(t);
                    frame = frame{1,1};
                    for n=1:40
                        if ismember(frame(n,1),ID_Marker)
                            single_marker = frame(n,:);
                            switch single_marker(1)
                                case ID_Marker(1)
                                    marker_estratti(1,:) = single_marker;
                                case ID_Marker(2)
                                    marker_estratti(2,:) = single_marker;
                                case ID_Marker(3)
                                    marker_estratti(3,:) = single_marker;
                                case ID_Marker(4)
                                    marker_estratti(4,:) = single_marker;
                                case ID_Marker(5)
                                    marker_estratti(5,:) = single_marker;
                                case ID_Marker(6)
                                    marker_estratti(6,:) = single_marker;
                                case ID_Marker(7)
                                    marker_estratti(7,:) = single_marker;
                                case ID_Marker(8)
                                    marker_estratti(8,:) = single_marker;
                                case ID_Marker(9)
                                    marker_estratti(9,:) = single_marker;
                                case ID_Marker(10)
                                    marker_estratti(10,:) = single_marker;
                                case ID_Marker(11)
                                    marker_estratti(11,:) = single_marker;
                                case ID_Marker(12)
                                    marker_estratti(12,:) = single_marker;
                                case ID_Marker(13)
                                    marker_estratti(13,:) = single_marker;
                                case ID_Marker(14)
                                    marker_estratti(14,:) = single_marker;
                                case ID_Marker(15)
                                    marker_estratti(15,:) = single_marker;
                                case ID_Marker(16)
                                    marker_estratti(16,:) = single_marker;
                                case ID_Marker(17)
                                    marker_estratti(17,:) = single_marker;
                                case ID_Marker(18)
                                    marker_estratti(18,:) = single_marker;
                                case ID_Marker(19)
                                    marker_estratti(19,:) = single_marker;
                                case ID_Marker(20)
                                    marker_estratti(20,:) = single_marker;
                            end
                        end
                    end
                    newdata{t,1} = marker_estratti;
                end
                
                Dataset_Estratto{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = newdata;
                newdata = [];
                
            catch
                Dataset_Estratto{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = [];
            end
        end
    end
    rmpath(pathname);
end

%Problemi coi marker
i=22;
j=4;
k=2;
Dataset_Estratto{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = [];

emptycells = find(cellfun(@isempty,Dataset_Estratto));
Dataset_Estratto(emptycells) = []; 

toc

%% Rotazione dati in sistema di riferimento torace

% Definizione coordinate locali marker torace

r     = 43;  
Gamma = pi/6;
hMK1  = 45;  
hMK2  = 40;  
hMK3  = 35;  
hMK4  = 67; 
d1    = 5;

% asse x: alto
% asse y: destra
% asse z: uscente dal torace

StarLocalCoords = [r*cos(Gamma) r*sin(Gamma)  hMK1;... %MK1
                   r            0             hMK2;... %MK2
                   r*cos(Gamma) -r*sin(Gamma) hMK3;... %MK3
                   0            0             hMK4];   %MK4

% Rotazione dati

Dataset_Ruotato = cell(length(Dataset_Estratto),1);

for i=1:length(Dataset_Estratto)
    
    data = Dataset_Estratto{i};
    data_rotated = cell(length(data),1);

    for j=1:length(data)
        Frame_data = cell2mat(data(j));
        IDs = Frame_data(:,1);      % I marker sono ordinati secondo il file IDS_Definition originale
        Pos = Frame_data(:,2:end);
    
        ReferencePos = Pos(1:4,:);
        [ReferenceNMK, ~] = size(ReferencePos);
        if ReferenceNMK == 4 && ReferencePos(1,1)~=0 && ReferencePos(2,1)~=0 && ReferencePos(3,1)~=0 && ReferencePos(4,1)~=0
            [Ropt, dopt, ~, T_hom_inv] = OptimalRigidPose(StarLocalCoords, ReferencePos);
        end
        
        [MkNum, ~] = size(Frame_data);
        clear Rotated_frame_data
        Rotated_frame_data = [];
    
        for k = 1:MkNum
            if Pos(k,1) ~= 0
                rotatedv = T_hom_inv*([Pos(k,:) 1]');
                Rotated_frame_data(k,:) = rotatedv(1:3);
            else
                rotatedv = [Pos(k,:) 1]';
                Rotated_frame_data(k,:) = rotatedv(1:3);
            end
        end
        
        NewFrame = [IDs Rotated_frame_data];
        data_rotated{j,1} = NewFrame;
    end
    Dataset_Ruotato{i,1} = data_rotated;
end

save('Dataset_Ruotato','Dataset_Ruotato')

%% Suddivisione Dataset

%In questa parte si prende il dataset completo e lo si separa in tre parti,
%dividendo così i dati riguardanti i 3 corpi rigidi separati
%(braccio,avambraccio e mano)

tic

Dataset_Braccio = cell(length(Dataset_Ruotato),1);
Dataset_Avambraccio = cell(length(Dataset_Ruotato),1);
Dataset_Mano = cell(length(Dataset_Ruotato),1);

for i = 1:length(Dataset_Ruotato)
    movim = Dataset_Ruotato{i};
    cella_arm = cell(length(movim),1);
    cella_for = cell(length(movim),1);
    cella_hnd = cell(length(movim),1);
    for j = 1:length(movim)
        frame = cell2mat(movim(j));
        tmp_arm = [transpose(frame(5,2:4));  transpose(frame(6,2:4)); ...
                   transpose(frame(7,2:4));  transpose(frame(8,2:4)); ...
                   transpose(frame(9,2:4)); transpose(frame(10,2:4))];
        
        tmp_for = [transpose(frame(11,2:4)); transpose(frame(12,2:4)); ...
                   transpose(frame(13,2:4)); transpose(frame(14,2:4)); ...
                   transpose(frame(15,2:4)); transpose(frame(16,2:4))];
        
        tmp_hnd = [transpose(frame(17,2:4)); transpose(frame(18,2:4)); ... 
                   transpose(frame(19,2:4)); transpose(frame(20,2:4))];
        
        cella_arm{j,1} = tmp_arm;
        cella_for{j,1} = tmp_for;
        cella_hnd{j,1} = tmp_hnd;
    end
    Dataset_Braccio{i,1} = cella_arm;
    Dataset_Avambraccio{i,1} = cella_for;
    Dataset_Mano{i,1} = cella_hnd;
end
toc

save('Dataset_Braccio', 'Dataset_Braccio')
save('Dataset_Avambraccio', 'Dataset_Avambraccio')
save('Dataset_Mano', 'Dataset_Mano')