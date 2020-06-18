figure, 
for j = 1 : 100%size(Bigm,3)
    tmp = Bigm(:,:,j);
    for i = 1 : ndof
        subplot(3,4,i);hold on;plot(tmp(i,:),'LineWidth',1)
    end
    drawnow
end



figure, 
for j = 1 : size(Warped_Datasets)
    tmp = Warped_Datasets{j};
    for i = 1 : ndof
        subplot(3,4,i);hold on;plot(tmp(i,:),'LineWidth',1)
    end
    drawnow
end


figure, 
for j = 1 : size(Datasets)
    tmp = Datasets{j};
    for i = 1 : ndof
        subplot(3,4,i);hold on;plot(tmp(i,:))
    end
    drawnow
end


Sub_1 = [coeff_All(:,1) coeff_All(:,2)];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i)];
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


Sub_1 = [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3) coeff_All(:,4)];
Alphas = [];
Alphas1 = [];
for i = 1 : num_frames
    Sub_i = [Coeffs_1(:,i) Coeffs_2(:,i) Coeffs_3(:,i) Coeffs_4(:,i)];
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
