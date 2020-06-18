

Bigm_extended = zeros(21,571,2648);

for i = 1 : 571
    
    
matrix_i = squeeze(Bigm(:,i,:));
matrix_i_1 = circshift(matrix_i',-1)';
matrix_i_2 = circshift(matrix_i',-2)';
matrix_l = [matrix_i; matrix_i_1; matrix_i_2];

Bigm_extended(:,i,:) = matrix_l;
    
end






%%

differences = zeros(4,4,7);
for i = 1 : 7
    
    v1 = all_ds_syns(1:7  ,i)/norm(all_ds_syns(1:7  ,i));
    v2 = all_ds_syns(8:14 ,i)/norm(all_ds_syns(8:14 ,i));
    v3 = all_ds_syns(15:21,i)/norm(all_ds_syns(15:21,i));
    v4 = coeff_All(:,i);
    d_i = [];
    for j = 1:4
        for k = 1:4
            eval(['d_i(',num2str(j),',',num2str(k),') = subspacea(v',num2str(j),',v',num2str(k),');'])
        end
    end
    differences(:,:,i) = d_i;
max(max(rad2deg(d_i)))
end




%%


Sub_1 = [coeff_All(:,1) coeff_All(:,2) coeff_All(:,3) coeff_All(:,4) ];
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



