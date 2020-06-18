



X = 0; l = 0;
for i = 1 : 90%length(Filt_Datasets)
        tmp = Filt_Datasets{i};
        if length(tmp(3,:)) > l && ~isnan(tmp(3,1))
            l = length(tmp(3,:));
            X = i; 
        end
end

% warp everything w.r.t the ---------



figure, 
for j = 1
    tmp = Filt_Datasets{j};
    for i = 1 : 7
        subplot(3,4,i);hold on;plot(tmp(i,:),'LineWidth',1)
    end
    drawnow
end
