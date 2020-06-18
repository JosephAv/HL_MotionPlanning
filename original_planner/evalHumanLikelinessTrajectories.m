clear all
clc

mat = dir('data/10_syns_incremental/*.mat'); 
P_total = zeros(268,7,size(mat,1));
V_total = zeros(267,7,size(mat,1));
A_total = zeros(266,7,size(mat,1));
J_total = zeros(265,7,size(mat,1));

costf = zeros(length(mat),10);

Errors_in_pose = [];
for q = 1:length(mat) 
    name = ['data/10_syns_incremental/', mat(q).name];
    load(name); 
    try 
        p_tmp = (Traj_opt);
        v_tmp = diff(Traj_opt);
        a_tmp = diff(v_tmp);
        j_tmp = diff(a_tmp);
    catch
        %v_tmp = [];
    end
    
    P_total(:,:,q) = p_tmp;
    V_total(:,:,q) = v_tmp;
    A_total(:,:,q) = a_tmp;
    J_total(:,:,q) = j_tmp;
    
    Errors_in_pose = [Errors_in_pose P_err_opt];
    
    try
        cost_i = [F_opt_all F_opt_all(end)*ones(1,10-length(F_opt_all))];
    catch
        cost_i = zeros(1,10);
    end
    costf(q,:) = cost_i;
end

mean(Errors_in_pose)
std(Errors_in_pose)

figure, 
for j = 1 : size(P_total,3)
    tmp = P_total(:,:,j);
    for i = 1 : 7
        subplot(2,4,i);hold on;plot(tmp(:,i))
        xlim([0 267])
        xticks([0 66 133 200 267])
        xticklabels({'0','25','50','75','100'})
        grid on
    end
    drawnow
end

figure, 
for j = 1 : size(V_total,3)
    tmp = V_total(:,:,j);
    for i = 1 : 7
        subplot(2,4,i);hold on;plot(tmp(:,i))
        xlim([0 267])
        xticks([0 66 133 200 267])
        xticklabels({'0','25','50','75','100'})
        %ylim([-0.02 0.02])
        yticks([-0.02 : 0.005 : 0.02])
        yticklabels({'-20' '-15' '-10' '-5' '0' '5' '10' '15' '20'})
        grid on
    end
    drawnow
end

figure, 
for j = 1 : size(A_total,3)
    tmp = A_total(:,:,j);
    for i = 1 : 7
        subplot(2,4,i);hold on;plot(tmp(:,i))
    end
    drawnow
end

figure, 
for j = 1 : size(J_total,3)
    tmp = J_total(:,:,j);
    for i = 1 : 7
        subplot(2,4,i);hold on;plot(tmp(:,i),'*-')
    end
    drawnow
end

jerks = reshape(squeeze(vecnorm(J_total,2,1)),84,1);
figure,plot(jerks/265,'*')

mean(jerks/265)
std((jerks/265))


%%

load Data/P_err_Zero



costf = [P_err_Zero' costf];
costf = [costf costf(:,end-5:end)];
%costf = costf([2:3, 5:6, 8:9, 11:12],:)
figure, plot(costf')

figure, bar((mean(costf)))



figure, bar(     mean(diff(costf')') /      sum(mean(diff(costf')'))           )
hold on, errorbar(mean(diff(costf')') /      sum(mean(diff(costf')')) ,   std(diff(costf')') /      sum(mean(diff(costf')'))         )
xlabel('fPC')
ylabel('Residual Cost Function')
 xticks([1:10])
 yticks([0:0.1:1])
 xticklabels([{'1'},{'2'},{'3'},{'4'},{'5'},{'6'},{'7'},{'8'},{'9'},{'10'},{'11'},{'12'},{'13'},{'14'},{'15'}])
 yticklabels([{'0%'},{'10%'},{'20%'},{'30%'},{'40%'},{'50%'},{'60%'},{'70%'},{'80%'},{'90%'},{'100%'}])
grid on

