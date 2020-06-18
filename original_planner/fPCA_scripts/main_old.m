close all
clear all
clc

%% Let's start
addpath('dynamic_time_warping')


Ntask = 30;

list_of_subj;
               
Nsubj = numel(Subj);
Maxnumrep = 6;
Datasets = cell(Nsubj*Ntask*Maxnumrep,1);

for i = 1 : Nsubj
    for j = 1 : Ntask
        for k = 1 : Maxnumrep
            try
                filename = ['../Collected_data/' Subj{i} '/EstimatedAngles/EstimatedQArm' AnglesName{i} '_' num2str(j) '_' num2str(k) '.mat'];
                tmp = load(filename);
                tmp = tmp.EstimatedQ;
                
                if i == 22 && j == 4 && k == 2
                    Datasets{(i-1)*30 + (j-1)*Maxnumrep + k} = [];

                else
                    Datasets{(i-1)*30 + (j-1)*Maxnumrep + k} = tmp(:,200:end-200);
                end
                
            catch
                Datasets{(i-1)*30 + (j-1)*Maxnumrep + k} = [];
            end
        end
    end
end

emptycells = find(cellfun(@isempty,Datasets));
Datasets(emptycells) = []; 



% figure;
% 
% for i = 1 : size(Datasets)
%     tmp = Datasets{i};
%     subplot(2,4,1);hold on;plot(tmp(1,:))
%     subplot(2,4,2);hold on;plot(tmp(2,:))
%     subplot(2,4,3);hold on;plot(tmp(3,:))
%     subplot(2,4,4);hold on;plot(tmp(4,:))
%     subplot(2,4,5);hold on;plot(tmp(5,:))
%     subplot(2,4,6);hold on;plot(tmp(6,:))
%     subplot(2,4,7);hold on;plot(tmp(7,:))
%     drawnow
% end



%filtering data and remove divergent samples
Filt_Datasets = cell(length(Datasets),1);
Len = [];
for i = 1 : numel(Datasets)
    
     tmp = Datasets{i};
     if 1%abs(tmp(1,1)-tmp(1,end))<1 && abs(tmp(6,1)-tmp(6,end))<1 && abs(tmp(7,1)-tmp(7,end))<1
         for j = 1 : 7

            angle_rad = smooth(tmp(j,:),50);
            %angle_rad = angle_rad - mean(angle_rad);
            tmp(j,:) = angle_rad;%wrapTo2Pi( angle_rad) ;%- 2*pi*floor( (angle_rad+pi)/(2*pi) ); 
         end
         Filt_Datasets{i} = tmp;

     else
         Filt_Datasets{i} = [];
     end

    %[a,b] = size(tmp);
    %Len = [Len b];
end
emptycells = find(cellfun(@isempty,Filt_Datasets));
Filt_Datasets(emptycells) = []; 

% figure;
% 
% for i = 1 : size(Filt_Datasets)
%     tmp = Filt_Datasets{i};
%     subplot(2,4,1);hold on;plot(tmp(1,:))
%     subplot(2,4,2);hold on;plot(tmp(2,:))
%     subplot(2,4,3);hold on;plot(tmp(3,:))
%     subplot(2,4,4);hold on;plot(tmp(4,:))
%     subplot(2,4,5);hold on;plot(tmp(5,:))
%     subplot(2,4,6);hold on;plot(tmp(6,:))
%     subplot(2,4,7);hold on;plot(tmp(7,:))
%     drawnow
% end





%% PCA ANALYSIS

%ScriptPCA7gdl

%% PCA ANALYSIS TIME_SEGMENTED

% ScriptPCA7gdl_time_segmented


%% WARPING

%warping on dof 3
% X = 0; Y = 0; l = Inf;
% for i = 1 : length(tasks)
%     for j = 1 : 3
%         tmp = Segmented_Datasets{i,j};
%         if length(tmp(3,:)) < l && tmp(3,10) ~= 0
%             l = length(tmp(3,:));
%             X = i; J = j;
%         end
%     end
% end
X = 0; l = Inf;
for i = 1 : length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if length(tmp(3,:)) < l && ~isnan(tmp(3,1))
            l = length(tmp(3,:));
            X = i; 
        end
end

%warp everything w.r.t the ---------

%RefSign = Segmented_Datasets{X,J};
RefSign =  Filt_Datasets{1};
Warped_Datasets = cell(length(Filt_Datasets),1);

for i = 1 : length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if ~isnan(tmp(3,10))
            tmp1 = TimeWarpingSingleND(RefSign,tmp);
        else
            tmp1 = RefSign*NaN;
        end
        Warped_Datasets{i} = tmp1;
        disp(['warping done vector ', num2str(i)]);
end
% 
% figure;
% 
% for i = 1 : size(Warped_Datasets)
%     tmp = Warped_Datasets{i};
%     subplot(2,4,1);hold on;plot(tmp(1,:))
%     subplot(2,4,2);hold on;plot(tmp(2,:))
%     subplot(2,4,3);hold on;plot(tmp(3,:))
%     subplot(2,4,4);hold on;plot(tmp(4,:))
%     subplot(2,4,5);hold on;plot(tmp(5,:))
%     subplot(2,4,6);hold on;plot(tmp(6,:))
%     subplot(2,4,7);hold on;plot(tmp(7,:))
%     drawnow
% end

save Datasets Datasets
save Filt_Datasets Filt_Datasets
save Warped_Datasets Warped_Datasets

%% Remove the mean values

Warped_Datasets_mean = cell(length(Warped_Datasets),1);

ndata = size(Warped_Datasets_mean,1);

MEAN = zeros(ndata,7);
for i = 1 : ndata
    
        tmp = Warped_Datasets{i};
        for k = 1 : 7
            MEAN(i,k) = mean(tmp(k,:));
            tmp(k,:) = tmp(k,:)-mean(tmp(k,:));
        end
        Warped_Datasets_mean{i} = tmp;
        
    
end
figure;

for i = 1 : size(Warped_Datasets_mean)
    tmp = Warped_Datasets_mean{i};
    subplot(2,4,1);hold on;plot(tmp(1,:))
    subplot(2,4,2);hold on;plot(tmp(2,:))
    subplot(2,4,3);hold on;plot(tmp(3,:))
    subplot(2,4,4);hold on;plot(tmp(4,:))
    subplot(2,4,5);hold on;plot(tmp(5,:))
    subplot(2,4,6);hold on;plot(tmp(6,:))
    subplot(2,4,7);hold on;plot(tmp(7,:))
    drawnow
end

save Warped_Datasets_mean Warped_Datasets_mean
save MEAN MEAN

num_dofs = 7;
num_frames = size(Warped_Datasets_mean{1},2);
num_samples = size(Warped_Datasets_mean,1);

Bigm = zeros(num_dofs,num_frames,num_samples);

for i = 1 : num_samples
    Bigm(:,:,i) = Warped_Datasets{i};
end

save Bigm Bigm

%% PCA all



PCA_Dataset_All = []; 

for i = 1:length(Warped_Datasets)
    PCA_Dataset_All = [PCA_Dataset_All Warped_Datasets{i}];
end



[coeff_All,score_All,latent_All,tsquared_All,explained_All] = pca(PCA_Dataset_All');

%% PCA in time

%pca per frame
Coeffs_1 = []; Coeffs_2 = []; Coeffs_3 = [];

Explained_1 = []; Explained_2 = []; Explained_3 = [];
for i = 1 : num_frames
    
    dataset_frame =  squeeze(Bigm(:,i,:));
    [coeff_frame,~,~,~,explained_frame] = pca(dataset_frame');
    if coeff_frame(7,1)>=0
        Coeffs_1 = [Coeffs_1 coeff_frame(:,1)]; 
    else
        Coeffs_1 = [Coeffs_1 -coeff_frame(:,1)]; 
    end
    
    if coeff_frame(7,2)>=0
        Coeffs_2 = [Coeffs_2 coeff_frame(:,2)]; 
    else
        Coeffs_2 = [Coeffs_2 -coeff_frame(:,2)]; 
    end
    
    if coeff_frame(7,3)>=0
        Coeffs_3 = [Coeffs_3 coeff_frame(:,3)]; 
        a=a+1;
    else
        Coeffs_3 = [Coeffs_3 -coeff_frame(:,3)]; 
        b=b+1;
    end
    Explained_1 = [Explained_1 explained_frame(1)]; 
    Explained_2 = [Explained_2 explained_frame(2)]; 
    Explained_3 = [Explained_3 explained_frame(3)];
end

figure, plot(Explained_1,'*r'), hold on, plot(Explained_2,'*g'), hold on,...
    plot(Explained_3,'*b'), hold on, plot(Explained_1+Explained_2+Explained_3,'*k')

figure, 
subplot(2,4,1);hold on;plot(Coeffs_1(1,:),'*r'),ylim([-1 1])
subplot(2,4,2);hold on;plot(Coeffs_1(2,:),'*r'),ylim([-1 1])
subplot(2,4,3);hold on;plot(Coeffs_1(3,:),'*r'),ylim([-1 1])
subplot(2,4,4);hold on;plot(Coeffs_1(4,:),'*r'),ylim([-1 1])
subplot(2,4,5);hold on;plot(Coeffs_1(5,:),'*r'),ylim([-1 1])
subplot(2,4,6);hold on;plot(Coeffs_1(6,:),'*r'),ylim([-1 1])
subplot(2,4,7);hold on;plot(Coeffs_1(7,:),'*r'),ylim([-1 1])

figure, 
for i = 1 : 7
    eval(['subplot(2,4,',num2str(i),');hold on;plot(Coeffs_1(',num2str(i),',:)),ylim([-1 1])'])
end

figure, 
subplot(2,4,1);hold on;plot(Coeffs_2(1,:),'*r'),ylim([-1 1])
subplot(2,4,2);hold on;plot(Coeffs_2(2,:),'*r'),ylim([-1 1])
subplot(2,4,3);hold on;plot(Coeffs_2(3,:),'*r'),ylim([-1 1])
subplot(2,4,4);hold on;plot(Coeffs_2(4,:),'*r'),ylim([-1 1])
subplot(2,4,5);hold on;plot(Coeffs_2(5,:),'*r'),ylim([-1 1])
subplot(2,4,6);hold on;plot(Coeffs_2(6,:),'*r'),ylim([-1 1])
subplot(2,4,7);hold on;plot(Coeffs_2(7,:),'*r'),ylim([-1 1])

figure, 
subplot(2,4,1);hold on;plot(Coeffs_3(1,:),'*r'),ylim([-1 1])
subplot(2,4,2);hold on;plot(Coeffs_3(2,:),'*r'),ylim([-1 1])
subplot(2,4,3);hold on;plot(Coeffs_3(3,:),'*r'),ylim([-1 1])
subplot(2,4,4);hold on;plot(Coeffs_3(4,:),'*r'),ylim([-1 1])
subplot(2,4,5);hold on;plot(Coeffs_3(5,:),'*r'),ylim([-1 1])
subplot(2,4,6);hold on;plot(Coeffs_3(6,:),'*r'),ylim([-1 1])
subplot(2,4,7);hold on;plot(Coeffs_3(7,:),'*r'),ylim([-1 1])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sub_1 = [mean(Coeffs_1')'];
Alphas = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i)];
    alpha = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
end
figure, plot(Alphas,'*r')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')'];
Alphas = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i)];
    alpha = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
end
figure, plot(Alphas,'*r')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')' mean(Coeffs_3')'];
Alphas = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i)];
    alpha = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
end
figure, plot(Alphas,'*r')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sub_1 = [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3)];
Alphas = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i)];
    alpha = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
end
figure, plot(Alphas,'*r')
%% Functional PCA

%functional_analysis


