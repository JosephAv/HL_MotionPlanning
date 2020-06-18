%close all
clear all
clc

plotsDirName = 'LUH/plots/';
addpath('dynamic_time_warping')

%% Let's start


Ntask = 30;

%list_of_subj;
               
Nsubj = 6;
Maxnumrep = 6;
ndof=7;

Datasets = cell(Nsubj*Ntask*Maxnumrep,1);

for i = 4 : 9 %subj
    for j = 1 : Ntask
        for k = 1 : Maxnumrep
            try
                filename = ['LUH/data_7_warped/Subj_', num2str(i), '_Task_', num2str(j+1), '_Rep_', num2str(k),'.mat'];
                tmp = load(filename);
                tmp = tmp.EstimatedQ;
                
                if tmp == zeros(ndof,1)
                    continue
                end
                
                Datasets{(i-4)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = tmp(:,1:end); %200 150

            catch
                %disp 'gesù'
                Datasets{(i-4)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = [];
            end
        end
    end
end

emptycells = find(cellfun(@isempty,Datasets));
Datasets(emptycells) = []; 

% figure, 
% for j = 1 : size(Datasets)
%     tmp = Datasets{j};
%     for i = 1 : ndof
%         subplot(3,4,i);hold on;plot(tmp(i,:))
%     end
%     drawnow
% end

%filtering data and remove divergent samples
Filt_Datasets = cell(length(Datasets),1);
Len = [];
for i = 1 : numel(Datasets)
    
     tmp = Datasets{i};
     if sum(tmp(4,:)<-0.5) == 0 && tmp(2,100) > -1.55 ... 
            && abs(mean(tmp(1,1:10))-mean(tmp(1,end-10:end))) <pi && abs(mean(tmp(1,:)))<pi ...
            && abs(mean(tmp(2,1:10))-mean(tmp(2,end-10:end))) <pi && abs(mean(tmp(2,:)))<pi ...
            && abs(mean(tmp(3,1:10))-mean(tmp(3,end-10:end))) <pi && abs(mean(tmp(3,:)))<pi ...
            && abs(mean(tmp(4,1:10))-mean(tmp(4,end-10:end))) <pi && abs(mean(tmp(4,:)))<pi ...
            && abs(mean(tmp(5,1:10))-mean(tmp(5,end-10:end))) <pi && abs(mean(tmp(5,:)))<pi ...
            && abs(mean(tmp(6,1:10))-mean(tmp(6,end-10:end))) <pi && abs(mean(tmp(6,:)))<pi ...
            && abs(mean(tmp(7,1:10))-mean(tmp(7,end-10:end))) <pi && abs(mean(tmp(7,:)))<pi ...
            
         for j = 1 : ndof
            angle_rad = smooth(tmp(j,:),1); %50
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

figure, 
for j = 1 : size(Filt_Datasets)
    tmp = Filt_Datasets{j};
    for i = 1 : ndof
        subplot(3,4,i);hold on;plot(tmp(i,:))
    end
    drawnow
end

%% WARPING

%select the shortest sample (we are not using it...)

X = 0; l = 0;
for i = 1 : length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if length(tmp(3,:)) > l && ~isnan(tmp(3,1))
            l = length(tmp(3,:));
            X = i; 
        end
end

% warp everything w.r.t the ---------


% Warped_Datasets = cell(length(Filt_Datasets),1);
% ref = Filt_Datasets{1};
% for i = 1 : 87
%     time     = 1 : length(ref);
%     timen     = 1 : length(Filt_Datasets{i});
%     nel      = length(timen);
%     timenew  = linspace(time(1),time(end),ceil(nel));
% 
%     v1_new = resample(timeseries(Filt_Datasets{i},timenew),time);
%     Warped_Datasets{i} = squeeze(v1_new.Data);
% end

RefSign =  Filt_Datasets{1};
Warped_Datasets = cell(length(Filt_Datasets),1);

% % tic
% % parfor i = 1 : length(Filt_Datasets)
% %         tmp = Filt_Datasets{i};
% %         if ~isnan(tmp(3,10))
% %             tmp1 = TimeWarpingSingleMxN(RefSign,tmp);
% %         else
% %             tmp1 = RefSign*NaN;
% %         end
% %         Warped_Datasets{i} = tmp1(:,1:end);
% %         disp(['warping done vector ', num2str(i)]);
% % end
% % disp(['Warping complete ']);
% % toc


Warped_Datasets = Filt_Datasets;


% figure, 
% for j = 1 : size(Warped_Datasets)
%     tmp = Warped_Datasets{j};
%     for i = 1 : ndof
%         subplot(3,4,i);hold on;plot(tmp(i,:),'LineWidth',1)
%     end
%     drawnow
% end

save LUH/Savings/Datasets Datasets
save LUH/Savings/Filt_Datasets Filt_Datasets
save LUH/Savings/Warped_Datasets Warped_Datasets

%% Remove the mean values

Warped_Datasets_mean = cell(length(Warped_Datasets),1);

ndata = size(Warped_Datasets_mean,1);

MEAN = zeros(ndata,ndof);
for i = 1 : ndata
    
        tmp = Warped_Datasets{i};
        for k = 1 : ndof
            MEAN(i,k) = mean(tmp(k,:));
            tmp(k,:) = tmp(k,:)-mean(tmp(k,:));
        end
        Warped_Datasets_mean{i} = tmp;
        
    
end


% figure, 
% for j = 1 : size(Warped_Datasets_mean)
%     tmp = Warped_Datasets_mean{j};
%     for i = 1 : ndof
%         subplot(3,4,i);hold on;plot(tmp(i,:))
%     end
%     drawnow
% end
save LUH/Savings/Warped_Datasets_mean Warped_Datasets_mean
save LUH/Savings/MEAN MEAN

%% Build Bigm
% load Savings/Bigm
%  load LUH/Savings/Warped_Datasets
ndof = 7;
cutt_frames = 0; %1681 + 500 - 1;%399
%cutt_frames = 1681 + 500 - 1; %  1681 + 500 - 1  luh %% 200+1247-1
%cutt_frames = 600 -1 ;%%%800 + 1881 -1;con edo

num_dofs = ndof;
num_frames = size(Warped_Datasets{1},2)-cutt_frames; %num_frames = 4001;
num_samples = size(Warped_Datasets,1);%-4;

Bigm = zeros(num_dofs,num_frames,num_samples);
k=0;
for i = 1:length(Warped_Datasets)% [1:12 14:114 116:137 139:438 440:476]%1:num_samples %% remove outliers
    k = k+1;
    tmp = Warped_Datasets{i};
    Bigm(:,:,k) = tmp(:,:);%origin 500:4500 con edo 800:3800
    
    %Bigm(:,:,i) = tmp(:,200:1800);%pisa data? 
    %Bigm(:,:,i) = tmp(:,:);
end

%save LUH/Savings/Bigm Bigm

%% PCA all
%%%%%% calculate PCA of the whole dataset %%%%%%

PCA_Dataset_All = []; 
 %  Filt_Datasets
tic
for i = 1:length(Warped_Datasets) %[1:12 14:114 116:137 139:438 440:476]   %[1:12 14:114 116:137 139:438 440:476] %1:length(Warped_Datasets)
    tmp = Warped_Datasets{i};
    tmp1=tmp(:,1:2000);%500:4500
    
    PCA_Dataset_All = [PCA_Dataset_All tmp1];
end
toc

PCA_Dataset_All = PCA_Dataset_All(:,:);

mean_pose = 0*median(PCA_Dataset_All')';
PCA_Dataset_All = PCA_Dataset_All - repmat(mean_pose,1,size(PCA_Dataset_All,2));

[coeff_All,score_All,latent_All,tsquared_All,explained_All] = pca(PCA_Dataset_All');

all_ds_syns = coeff_All;
save LUH/Savings/all_ds_syns all_ds_syns
save LUH/Savings/mean_pose mean_pose

%% PCA in time

%%%%%% define empty vars %%%%%%
for j = 1 : num_dofs
    eval(['Coeffs_',num2str(j),' = [];' ])
    eval(['Explained_',num2str(j),' = [];' ])
end
    
%%%%%% pca per frame %%%%%%
for i =  1: num_frames
    %coeff_frame_old = coeff_frame;
    dataset_frame =  squeeze(Bigm(:,i,:));
    [coeff_frame,~,~,~,explained_frame] = pca(dataset_frame'); 
        
        
    for j = 1 : num_dofs %for each pc
        
        if 1%abs(coeff_frame(5,j)-coeff_frame_old(5,j)) <= 0.01
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ]); 
            
        else
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' -coeff_frame(:,',num2str(j),')];' ]); 
        end
        
        %eval(['Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ])
        eval(['Explained_',num2str(j),' = [Explained_',num2str(j),' explained_frame(',num2str(j),')];' ])
    end
    
end

%%%%%% plot variances %%%%%%
figure, plot(Explained_1,'*r'), hold on, plot(Explained_2,'*g'), hold on,...
    plot(Explained_3,'*b'), hold on, plot(Explained_1+Explained_2+Explained_3,'*k')
title('Explained Variances'), legend('S1','S2','S3','S1+S2+S3')
plotName = 'Expl_variances'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the t.v. synergies coefficients %%%%%%
for j = 1 : 3% 7
    figure,     
    for i = 1 : ndof
      subplot(3,4,i);hold on;plot(eval(['Coeffs_',num2str(j),'(i,:)']),'r*'),
      ylim([-1 1]),xlim([0 length(eval(['Coeffs_',num2str(j),'(i,:)']))]);
    end
    
    text(-1000,4,['Time variant values of Syn ',num2str(j),' coefficients.'],'FontSize',14,'FontWeight','bold')
    plotName = ['Syn_' num2str(j) '_tv_coeffs']; subfolder = 'pca/';
    saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
    saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
end
% % % % 
% % % % %%%%%% plot the s1 alphas %%%%%%
% % % % 
% % % % Sub_1 = [mean(Coeffs_1')'];
% % % % Alphas = [];
% % % % Alphas1 = [];
% % % % for i = 1 : num_frames
% % % %     Sub_i = [Coeffs_1(:,i)];
% % % %     alpha = subspacea(Sub_1, Sub_i);
% % % %     alpha1 = subspace(Sub_1, Sub_i);
% % % %     
% % % %     Alphas = [Alphas alpha];
% % % %     Alphas1 = [Alphas1 alpha1];
% % % % end
% % % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % % title('Subspace similarity between the first mean syns and the correspondent t.d. syns')
% % % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % % 
% % % % %%%%%% plot the s2 alphas %%%%%%
% % % % 
% % % % Sub_1 = [mean(Coeffs_2')'];
% % % % Alphas = [];
% % % % Alphas1 = [];
% % % % for i = 1 : num_frames
% % % %     Sub_i = [Coeffs_2(:,i)];
% % % %     alpha = subspacea(Sub_1, Sub_i);
% % % %     alpha1 = subspace(Sub_1, Sub_i);
% % % %     
% % % %     Alphas = [Alphas alpha];
% % % %     Alphas1 = [Alphas1 alpha1];
% % % % end
% % % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % % title('Subspace similarity between the second mean syns and the correspondent t.d. syns')
% % % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % % 
% % % % %%%%%% plot the s3 alphas %%%%%%
% % % % 
% % % % Sub_1 = [mean(Coeffs_3')'];
% % % % Alphas = [];
% % % % Alphas1 = [];
% % % % for i = 1 : num_frames
% % % %     Sub_i = [Coeffs_3(:,i)];
% % % %     alpha = subspacea(Sub_1, Sub_i);
% % % %     alpha1 = subspace(Sub_1, Sub_i);
% % % %     
% % % %     Alphas = [Alphas alpha];
% % % %     Alphas1 = [Alphas1 alpha1];
% % % % end
% % % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % % title('Subspace similarity between the third mean syns and the correspondent t.d. syns')
% % % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % % 
% % % % 
% % % % %%%%%% plot the s1-s2 alphas %%%%%%
% % % % 
% % % % Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')'];
% % % % Alphas = [];
% % % % Alphas1 = [];
% % % % for i = 1 : num_frames
% % % %     Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i)];
% % % %     alpha = subspacea(Sub_1, Sub_i);
% % % %     alpha1 = subspace(Sub_1, Sub_i);
% % % %     
% % % %     Alphas = [Alphas alpha];
% % % %     Alphas1 = [Alphas1 alpha1];
% % % % end
% % % % w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % % title('Subspace similarity between the first two mean syns and the correspondent t.d. syns')
% % % % plotName = 'Alphas_2_mean'; subfolder = 'pca/';
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % % 
% % % % %%%%%% plot the s1-s2-s3 alphas %%%%%%
% % % % 
% % % % Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')' mean(Coeffs_3')'];
% % % % Alphas = [];
% % % % Alphas1 = [];
% % % % for i = 1 : num_frames
% % % %     Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i)];
% % % %     alpha = subspacea(Sub_1, Sub_i);
% % % %     alpha1 = subspace(Sub_1, Sub_i);
% % % %     
% % % %     Alphas = [Alphas alpha];
% % % %     Alphas1 = [Alphas1 alpha1];
% % % % end
% % % % w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % % title('Subspace similarity between the first three mean syns and the correspondent t.d. syns')
% % % % plotName = 'Alphas_3_mean'; subfolder = 'pca/';
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the s1 alphas w.r.t. coeff_all %%%%%%

Sub_1 = [coeff_All(:,1) ];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) ];
    alpha = subspacea(Sub_1, Sub_i);
    alpha1 = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
    Alphas1 = [Alphas1 alpha1];
end
w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
title('Subspace similarity between the first all-data syns and the correspondent t.d. syns')
plotName = 'Alphas_3_all'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the s2 alphas w.r.t. coeff_all %%%%%%

Sub_1 = [coeff_All(:,2)];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_2(:,i)];
    alpha = subspacea(Sub_1, Sub_i);
    alpha1 = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
    Alphas1 = [Alphas1 alpha1];
end
w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
title('Subspace similarity between the second all-data syns and the correspondent t.d. syns')
plotName = 'Alphas_3_all'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the s3 alphas w.r.t. coeff_all %%%%%%

Sub_1 = [coeff_All(:,3)];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_3(:,i)];
    alpha = subspacea(Sub_1, Sub_i);
    alpha1 = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
    Alphas1 = [Alphas1 alpha1];
end
w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
title('Subspace similarity between the third all-data syns and the correspondent t.d. syns')
plotName = 'Alphas_3_all'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the s1-s2-s3 alphas w.r.t. coeff_all %%%%%%

Sub_1 = [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3)];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i)];
    alpha = subspacea(Sub_1, Sub_i);
    alpha1 = subspace(Sub_1, Sub_i);
    
    Alphas = [Alphas alpha];
    Alphas1 = [Alphas1 alpha1];
end
w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
title('Subspace similarity between the first three all-data syns and the correspondent t.d. syns')
plotName = 'Alphas_3_all'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

%%%%%% plot the s1-s2-s3-s4 alphas w.r.t. coeff_all %%%%%%

% Sub_1 = [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3) coeff_All(:,4)];
% Alphas = [];
% Alphas1 = [];
% for i = 1 : num_frames
%     Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i) Coeffs_4(:,i)];
%     alpha = subspacea(Sub_1, Sub_i);
%     alpha1 = subspace(Sub_1, Sub_i);
%     
%     Alphas = [Alphas alpha];
%     Alphas1 = [Alphas1 alpha1];
% end
% w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% title('Subspace similarity between the first three all-data syns and the correspondent t.d. syns')
% plotName = 'Alphas_3_all'; subfolder = 'pca/';
% saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));


%mean_syns = [mean(Coeffs_1')' mean(Coeffs_2')' mean(Coeffs_3')' mean(Coeffs_4')' mean(Coeffs_5')' mean(Coeffs_6')' mean(Coeffs_7')'];


%save LUH/Savings/mean_syns mean_syns
%save LUH/Savings/WSPACE_LUH
%% Functional PCA

%functional_analysis

%% Workspace analysis

% load (['UpperLimbParametersDEFAndrea.mat'])
% 
% UpperLimbParametersDEF(10) = UpperLimbParametersDEF(10)*2;
% 
% [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3)];
