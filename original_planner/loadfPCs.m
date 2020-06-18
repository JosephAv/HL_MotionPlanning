%load fPCS and mean functions in a cell structure


load('Savings/MEAN')
numdofs = 7;
for i = 1 : numdofs %7 DoFs
    eval( ['load  Savings/lista_pca_nr_dof',num2str(i),'']) %load structures
    eval( ['scores_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.componenti;']) %load scores
    eval( ['mean_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.media;']) %load mean functions

    eval( ['FD_dof',num2str(i),' = lista_pca_nr_dof',num2str(i),'.fd;']) %load PCs_struct
    
    for j = 1 : 15 %15 PCs
        eval( ['pc',num2str(j),'_dof',num2str(i),' = FD_dof',num2str(i),'.fPCA{',num2str(j),'};']) %extract single PCs
    end
        
end
clear i j
%figure, plot(pc1_dof1), hold on, plot(pc2_dof1), plot(pc7_dof1)
%legend('pc1','pc2','pc3')

%%
%select only the 'go' phase

numframes = 517;
%KineParameters = [180 180 -40  0  40  180 0 20 155 50 5 0 300 225]; 
KineParameters = [125.149108581732,148.431768191511,-115.973666354272,-19.8936354839336,28.6121705580542,158.985333633674,-7.78655708978049,31.1631329834081,155.888434145282,44.9424947316485,14.8290910319249,-8.35699021213840,276.725154360048,241.589264420293];

mean_traj = [mean_dof1 mean_dof2 mean_dof3 mean_dof4 mean_dof5 mean_dof6 mean_dof7];
mean_traj = mean_traj(250:numframes,:); %select only the reaching movement

% fPC1      = [pc1_dof1 pc1_dof2 pc1_dof3 pc1_dof4 pc1_dof5 pc1_dof6 pc1_dof7]; 
for i = 1 : 15
    eval ( [' fPC',num2str(i),' = [pc',num2str(i),'_dof1 pc',num2str(i),'_dof2 pc',num2str(i),'_dof3 pc',num2str(i),'_dof4 pc',num2str(i),'_dof5 pc',num2str(i),'_dof6 pc',num2str(i),'_dof7]; ' ])
    eval ( [' fPC',num2str(i),'  = fPC',num2str(i),'(250:numframes,:);']) %select only the reaching movement
end

numframes_old = numframes;
numframes = size(mean_traj,1);

