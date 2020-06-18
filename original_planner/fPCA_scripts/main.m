%close all
clear all
clc

plotsDirName = 'plots/';
addpath('dynamic_time_warping')

%% Let's start


Ntask = 30;

list_of_subj;
               
Nsubj = numel(Subj);
Maxnumrep = 6;
Datasets = cell(Nsubj*Ntask*Maxnumrep,1);

%selected_subj = randsample(1:33,20);

for i = 1:33
    for j = 1 : Ntask
        for k = 1 : Maxnumrep
            try
                filename = ['../Collected_data/' Subj{i} '/EstimatedAngles/EstimatedQArm' AnglesName{i} '_' num2str(j) '_' num2str(k) '.mat'];
                tmp = load(filename);
                tmp = tmp.EstimatedQ;
                
                if tmp == zeros(7,1)
                    continue
                end
                
                if i == 22 && j == 4 && k == 2
                    Datasets{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = [];

                else
                    Datasets{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = tmp(:,1:end); %200 150
                end
                
            catch
                Datasets{(i-1)*Ntask*Maxnumrep + (j-1)*Maxnumrep + k} = [];
            end
        end
    end
end

emptycells = find(cellfun(@isempty,Datasets));
Datasets(emptycells) = []; 

% figure, 
% for j = 1 : size(Datasets)
%     tmp = Datasets{j};
%     for i = 1 : 7
%         subplot(2,4,i);hold on;plot(tmp(i,:))
%     end
%     drawnow
% end

%filtering data and remove divergent samples
Filt_Datasets = cell(length(Datasets),1);
Len = [];

hmn=0;
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
            
         for j = 1 : 7

            angle_rad = smooth(tmp(j,:),10); %50
            %angle_rad = angle_rad - mean(angle_rad);
            tmp(j,:) = angle_rad;%wrapTo2Pi( angle_rad) ;%- 2*pi*floor( (angle_rad+pi)/(2*pi) ); 
         end
         Filt_Datasets{i} = tmp;

     else
         Filt_Datasets{i} = [];
         hmn = hmn + 1;
     end

    %[a,b] = size(tmp);
    %Len = [Len b];
end
emptycells = find(cellfun(@isempty,Filt_Datasets));
Filt_Datasets(emptycells) = []; 
% 
% figure, 
% for j = 1 : size(Filt_Datasets)
%     tmp = Filt_Datasets{j};
%     for i = 1 : 7
%         subplot(2,4,i);hold on;plot(tmp(i,:))
%     end
%     drawnow
% end

%% WARPING

%select the longest sample

X = 0; l = 0;
for i = 1 : length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if length(tmp(3,:)) > l && ~isnan(tmp(3,1))
            l = length(tmp(3,:));
            X = i; 
        end
end

% warp everything w.r.t the ---------

RefSign =  Filt_Datasets{X};
Warped_Datasets = cell(length(Filt_Datasets),1);
tic
parfor i = 1 : length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if ~isnan(tmp(3,10))
            tmp1 = TimeWarpingSingleND(RefSign,tmp);
        else
            tmp1 = RefSign*NaN;
        end
        Warped_Datasets{i} = tmp1(:,1:end);
        disp(['warping done vector ', num2str(i)]);
end
toc

%         figure, 
%         for j = 1 : size(Warped_Datasets,1)
%             tmp = Warped_Datasets{j};
%             for i = 1 : 7
%                 subplot(2,4,i);hold on;plot(tmp(i,:))
%             end
%             drawnow
%         end

save Savings/Datasets Datasets
save Savings/Filt_Datasets Filt_Datasets
save Savings/Warped_Datasets Warped_Datasets

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

% 
% figure, 
% for j = 1 : size(Warped_Datasets_mean)
%     tmp = Warped_Datasets_mean{j};
%     for i = 1 : 7
%         subplot(2,4,i);hold on;plot(tmp(i,:))
%     end
%     drawnow
% end
save Savings/Warped_Datasets_mean Warped_Datasets_mean
save Savings/MEAN MEAN

%% Build Bigm
%   load Savings/Bigm
%   load Savings/Warped_Datasets

cutt_frames = 399;
num_dofs = 7;
num_frames = size(Warped_Datasets{1},2)-cutt_frames;
num_samples = size(Warped_Datasets,1);

Bigm = zeros(num_dofs,num_frames,num_samples);

for i = 1 : num_samples
    tmp = Warped_Datasets{i};
    Bigm(:,:,i) = tmp(:,200:end-200);
end

save Savings/Bigm Bigm

Bigm_extended = zeros(21,571,2648);

for i = 1 : 571
    
    
matrix_i = squeeze(Bigm(:,i,:));
matrix_i_1 = circshift(matrix_i',-1)';
matrix_i_2 = circshift(matrix_i',-2)';
matrix_l = [matrix_i; matrix_i_1; matrix_i_2];

Bigm_extended(:,i,:) = matrix_l;
    
end
save Savings/Bigm_extended Bigm_extended

%Bigm = Bigm_extended;
%% PCA all
%%%%%% calculate PCA of the whole dataset %%%%%%

PCA_Dataset_All = []; 
 %  Filt_Datasets
% % % parfor i = 1:length(Warped_Datasets)
% % %     
% % %     tmp12 =  Warped_Datasets{i};
% % %     PCA_Dataset_All = [PCA_Dataset_All tmp12(:,200:end-200)];
% % %     disp (['done ' num2str(i) ' of ' size(Warped_Datasets{i},1)])
% % % end

% ----------------this is the one we had in the deliverable with everything inside---------------- 
for i = 1:size(Bigm,2)
    
    tmp12 =  squeeze(Bigm(:,i,:));
    PCA_Dataset_All = [PCA_Dataset_All tmp12];
    %disp (['done ' num2str(i) ' of ' size(Warped_Datasets{i},1)])
end

[coeff_All_all,score_All_all,latent_All_all,tsquared_All_all,explained_All_all] = pca(PCA_Dataset_All');

% ----------------here we test with the MEAN---------------- 

PCA_Dataset_All = []; 

%try with one
for i = 1:size(Bigm,3)
    
    tmp12 =  squeeze(Bigm(:,:,i));
    PCA_Dataset_All = [PCA_Dataset_All mean(tmp12,2)];
    %disp (['done ' num2str(i) ' of ' size(Warped_Datasets{i},1)])
end


PCA_Dataset_All = PCA_Dataset_All(:,:);
% 
% mean_pose = mean(PCA_Dataset_All')';
% PCA_Dataset_All = PCA_Dataset_All - repmat(mean_pose,1,size(PCA_Dataset_All,2));

[coeff_All_mean,score_All_mean,latent_All_mean,tsquared_All_mean,explained_All_mean] = pca(PCA_Dataset_All');


% ----------------here we test with the median---------------- 

PCA_Dataset_All = []; 

%try with one
for i = 1:size(Bigm,3)
    
    tmp12 =  squeeze(Bigm(:,:,i));
    PCA_Dataset_All = [PCA_Dataset_All median(tmp12,2)];
    %disp (['done ' num2str(i) ' of ' size(Warped_Datasets{i},1)])
end


PCA_Dataset_All = PCA_Dataset_All(:,:);
% 
% mean_pose = mean(PCA_Dataset_All')';
% PCA_Dataset_All = PCA_Dataset_All - repmat(mean_pose,1,size(PCA_Dataset_All,2));

[coeff_All_median,score_All_median,latent_All_median,tsquared_All_median,explained_All_median] = pca(PCA_Dataset_All');


%all_ds_syns = coeff_All;
%save Savings/all_ds_syns all_ds_syns
%save Savings/mean_pose mean_pose

% ----------------for our analysis we choose the mean---------------- 
coeff_All = coeff_All_mean;
%% PCA in time
plotsDirName = 'plots/';

%%%%%% define empty vars %%%%%%
for j = 1 : num_dofs
    eval(['Coeffs_',num2str(j),' = [];' ])
    eval(['Explained_',num2str(j),' = [];' ])
end
    
%%%%%% pca per frame %%%%%%
for i = 1 : num_frames
        dataset_frame =  squeeze(Bigm(:,i,:));
        [coeff_frame,~,~,~,explained_frame] = pca(dataset_frame'); 
        
    for j = 1 : num_dofs %for each synergy
        
        if  j==1 && coeff_frame(1,j)>=0
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ]); 
            
        elseif j==1
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' -coeff_frame(:,',num2str(j),')];' ]); 
        end
        
        if  j==2 && coeff_frame(7,j)<=0
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ]); 
            
        elseif j==2
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' -coeff_frame(:,',num2str(j),')];' ]); 
        end
        
        
        if  j==3 && coeff_frame(6,j)<=0
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ]); 
            
        elseif j>=3
            eval([ 'Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' -coeff_frame(:,',num2str(j),')];' ]); 
        end
        
        %eval(['Coeffs_',num2str(j),' = [Coeffs_',num2str(j),' coeff_frame(:,',num2str(j),')];' ])
        eval(['Explained_',num2str(j),' = [Explained_',num2str(j),' explained_frame(',num2str(j),')];' ])
    end
    
end

%%%%%% plot variances %%%%%%
figure, plot(Explained_1,'r','LineWidth',4), hold on, plot(Explained_2,'g','LineWidth',4), hold on,...
    plot(Explained_3,'b','LineWidth',4), hold on, plot(Explained_1+Explained_2+Explained_3,'k','LineWidth',4)
xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
xticklabels({'0','0.25','0.5','0.75','1'})
set(gca,'fontsize', 20);
axis([0 length(Explained_1) 0 100])
xlabel(' Time cycle')
ylabel(' Explained Variance [%]')
title('Explained Variances'), legend('S1','S2','S3','S1+S2+S3')

line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

plotName = 'Expl_variances'; subfolder = 'pca/';
saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));


%%%%%% plot the t.v. synergies coefficients %%%%%%
figure,hold on,
for j = 1 : 3% 7
    %figure,     
    for i = 1 : 7
        subplot(2,4,i);hold on;plot(eval(['Coeffs_',num2str(j),'(i,:)']),'LineWidth',4),ylim([-1 1]),xlim([0 length(eval(['Coeffs_',num2str(j),'(i,:)']))]);
      
        xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
        xticklabels({'0','0.25','0.5','0.75','1'})
        yticks([-1:0.2:1])

        set(gca,'fontsize', 20);
        axis([0 length(Explained_1) -1 1])
        xlabel(' Time cycle')
        ylabel(' PCs Coeff')
        
        grid on
    end
    
    plotName = ['Syn_' num2str(j) '_tv_coeffs']; subfolder = 'pca/';
    saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
    saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
    
end
    legend('S1','S2','S3')
    
    
%%%%%% plot the t.v. synergies coefficients... AGAIN%%%%%%
for i = 1 : 7
    for j = 1 : 3% 7
        figure(i);hold on;plot(eval(['Coeffs_',num2str(j),'(i,:)']),'LineWidth',4),ylim([-1 1]),xlim([0 length(eval(['Coeffs_',num2str(j),'(i,:)']))]);
      
        xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
        xticklabels({'0','0.25','0.5','0.75','1'})
        yticks([-1:0.2:1])

        set(gca,'fontsize', 20);
        axis([0 length(Explained_1) -1 1])
        xlabel(' Time cycle')
        ylabel(' PCs Coeff')
        line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
        line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
        line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
        line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

        grid on
    end
    
    plotName = ['Syn_' num2str(j) '_tv_coeffs_dof' num2str(i)]; subfolder = 'pca/';
    saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
    saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
    
end
%legend('S1','S2','S3')
    
    
for i = 1 :1
% % % 
% % % %%%%%% plot the s1 alphas %%%%%%
% % % 
% % % Sub_1 = [mean(Coeffs_1')'];
% % % Alphas = [];
% % % Alphas1 = [];
% % % for i = 1 : num_frames
% % %     Sub_i = [Coeffs_1(:,i)];
% % %     alpha = subspacea(Sub_1, Sub_i);
% % %     alpha1 = subspace(Sub_1, Sub_i);
% % %     
% % %     Alphas = [Alphas alpha];
% % %     Alphas1 = [Alphas1 alpha1];
% % % end
% % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % title('Subspace similarity between the first mean syns and the correspondent t.d. syns')
% % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % 
% % % %%%%%% plot the s2 alphas %%%%%%
% % % 
% % % Sub_1 = [mean(Coeffs_2')'];
% % % Alphas = [];
% % % Alphas1 = [];
% % % for i = 1 : num_frames
% % %     Sub_i = [Coeffs_2(:,i)];
% % %     alpha = subspacea(Sub_1, Sub_i);
% % %     alpha1 = subspace(Sub_1, Sub_i);
% % %     
% % %     Alphas = [Alphas alpha];
% % %     Alphas1 = [Alphas1 alpha1];
% % % end
% % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % title('Subspace similarity between the second mean syns and the correspondent t.d. syns')
% % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % 
% % % %%%%%% plot the s3 alphas %%%%%%
% % % 
% % % Sub_1 = [mean(Coeffs_3')'];
% % % Alphas = [];
% % % Alphas1 = [];
% % % for i = 1 : num_frames
% % %     Sub_i = [Coeffs_3(:,i)];
% % %     alpha = subspacea(Sub_1, Sub_i);
% % %     alpha1 = subspace(Sub_1, Sub_i);
% % %     
% % %     Alphas = [Alphas alpha];
% % %     Alphas1 = [Alphas1 alpha1];
% % % end
% % % w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % title('Subspace similarity between the third mean syns and the correspondent t.d. syns')
% % % plotName = 'Alphas_1_mean'; subfolder = 'pca/';
% % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % 
% % % 
% % % %%%%%% plot the s1-s2 alphas %%%%%%
% % % 
% % % Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')'];
% % % Alphas = [];
% % % Alphas1 = [];
% % % for i = 1 : num_frames
% % %     Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i)];
% % %     alpha = subspacea(Sub_1, Sub_i);
% % %     alpha1 = subspace(Sub_1, Sub_i);
% % %     
% % %     Alphas = [Alphas alpha];
% % %     Alphas1 = [Alphas1 alpha1];
% % % end
% % % w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % title('Subspace similarity between the first two mean syns and the correspondent t.d. syns')
% % % plotName = 'Alphas_2_mean'; subfolder = 'pca/';
% % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
% % % 
% % % %%%%%% plot the s1-s2-s3 alphas %%%%%%
% % % 
% % % Sub_1 = [mean(Coeffs_1')' mean(Coeffs_2')' mean(Coeffs_3')'];
% % % Alphas = [];
% % % Alphas1 = [];
% % % for i = 1 : num_frames
% % %     Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i)];
% % %     alpha = subspacea(Sub_1, Sub_i);
% % %     alpha1 = subspace(Sub_1, Sub_i);
% % %     
% % %     Alphas = [Alphas alpha];
% % %     Alphas1 = [Alphas1 alpha1];
% % % end
% % % w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
% % % title('Subspace similarity between the first three mean syns and the correspondent t.d. syns')
% % % plotName = 'Alphas_3_mean'; subfolder = 'pca/';
% % % saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
% % % saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

end %commented code...


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
%w=Alphas;figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'*r')
w=Alphas;figure,plot(rad2deg(Alphas'),'-r','LineWidth',2)%,hold on,plot(rad2deg(w),'*r')
%title('Subspace similarity between the first all-data syns and the correspondent t.d. syns')
plotName = 'Alpha_1_all'; subfolder = 'pca/';

xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
xticklabels({'0','0.25','0.5','0.75','1'})
yticks([0:10:60])
set(gca,'fontsize', 25);
axis([0 length(Explained_1) 0 60])
xlabel(' Time cycle')
ylabel(' Distance [deg]')
grid

line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
w1 = w;
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
w=Alphas;figure,plot(rad2deg(Alphas'),'-g','LineWidth',2)%,hold on,plot(rad2deg(w),'*r')
%title('Subspace similarity between the second all-data syns and the correspondent t.d. syns')
plotName = 'Alpha_2_all'; subfolder = 'pca/';

xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
xticklabels({'0','0.25','0.5','0.75','1'})
set(gca,'fontsize', 25);
yticks([0:10:60])
axis([0 length(Explained_1) 0 60])
xlabel(' Time cycle')
ylabel(' Distance [deg]')
grid

line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
w2 = w;

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
w=Alphas;figure,plot(rad2deg(Alphas'),'-b','LineWidth',2)%,hold on,plot(rad2deg(w),'*r')
%title('Subspace similarity between the third all-data syns and the correspondent t.d. syns')
plotName = 'Alpha_3_all'; subfolder = 'pca/';

xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
xticklabels({'0','0.25','0.5','0.75','1'})
yticks([0:10:60])
set(gca,'fontsize', 25);
axis([0 length(Explained_1) 0 60])
xlabel(' Time cycle')
ylabel(' Distance [deg]')
grid

line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));
w3 = w;

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
w=vecnorm(Alphas);figure,plot(rad2deg(Alphas')),hold on,plot(rad2deg(w),'k','LineWidth',2)
%title('Subspace similarity between the first three all-data syns and the correspondent t.d. syns')
plotName = 'Alphas_123_all'; subfolder = 'pca/';

xticks([1 length(Explained_1)/4 length(Explained_1)/2 3/4*length(Explained_1) length(Explained_1)])
xticklabels({'0','0.25','0.5','0.75','1'})
yticks([0:10:60])
set(gca,'fontsize', 25);
axis([0 length(Explained_1) 0 60])
xlabel(' Time cycle')
ylabel(' Distance [deg]')
grid

line([250 250],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([350 350],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([50 50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)
line([length(Explained_1)-50 length(Explained_1)-50],get(gca,'YLim'),'Color','blue','LineStyle','--','LineWidth',2)

saveas(gcf,strcat(plotsDirName,subfolder,'fig/',plotName,'.fig'));
saveas(gcf,strcat(plotsDirName,subfolder,'jpg/',plotName,'.jpg'));

mean_syns = [mean(Coeffs_1')' mean(Coeffs_2')' mean(Coeffs_3')' mean(Coeffs_4')' mean(Coeffs_5')' mean(Coeffs_6')' mean(Coeffs_7')'];

w123 = w;

%save Savings/mean_syns mean_syns

%save WspPCAintime



%% Functional PCA

%functional_analysis

%% Workspace analysis
% 
% load (['UpperLimbParametersDEFAndrea.mat'])
% 
% UpperLimbParametersDEF(10) = UpperLimbParametersDEF(10)*2;
% 
% [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3)];
% 
% save WspAnalys
