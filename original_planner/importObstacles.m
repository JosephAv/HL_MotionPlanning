

% % 
% % 
% % Obs_cart = [];
% % x = linspace(-60,20,30);
% % y = linspace(-250,300,30);
% % z = linspace(100,1000,30);
% % for i = x
% %     for j = y
% %         for k = z
% %             Obs_cart = [Obs_cart; [i j k]];
% %         end
% %     end
% % end
% % 
Joint_end = [0.6536 -0.4006 0.0704 0.7927 -1.3243 0.8680 0.6280]; %EstimatedQ(:,500) licia 1_1
P_end = calculateActualPose(KineParameters,Joint_end);
Joint_start = [0.0915 -0.7921 -0.6216 0.6544 -1.5007 0.0566 0.7626]; %EstimatedQ(:,100) licia 1_1
P_start = calculateActualPose(KineParameters,Joint_start);
% %  

c = [-13 220 350];
r = 50;

[xs,ys,zs] = sphere;
xs = r*xs + c(1);
ys = r*ys + c(2);
zs = r*zs + c(3);
%figure,plot3(xs,ys,zs,'*')

Obs_cart = reshape([xs,ys,zs],[441,3]);

plotScenarioV2(KineParameters, P_start, Joint_start, P_end, Joint_end, Obs_cart)


Obs{1} = Obs_cart;
