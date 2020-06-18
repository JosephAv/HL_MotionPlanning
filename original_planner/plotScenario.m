function plotScenario(kinPars, P_start, Joint_start, P_final, Joint_final, Obs)
    
if nargin < 4
    Obs = [];
elseif size(Obs,1) == 3
    Obs = Obs';
end

addpath fPCA_scripts/kinematics

MkSuppDimensions = [15 7 22.5 55]; 

figure('units','normalized','outerposition',[0 0 1 1])
hold on;
x_col = [255 0 0]/255;
y_col = [0 255 0]/255;
z_col = [0 0 255]/255;
coord_syst_colors_target = [x_col; y_col; z_col];

draw_coordinate_system(50,quat2rotm(P_start(4:7)),P_start(1:3),coord_syst_colors_target) %draw start pose
draw_coordinate_system(50,quat2rotm(P_final(4:7)),P_final(1:3),coord_syst_colors_target) %draw start pose

plot3(Obs(:,1),Obs(:,2),Obs(:,3),'*','Color',[255/255 0/255 0/255],'MarkerSize',4)
  
%%%%%%%%%%%%%%%%%%%%%% Reference Sys Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3([0 30],[0 0],[0 0],'r','LineWidth',2)
plot3([0 0],[0 30],[0 0],'g','LineWidth',2)
plot3([0 0],[0 0],[0 30],'b','LineWidth',2)

%% 

%%%%%%%%%%%%%%%% Estimated Positions of Joints %%%%%%%%%%%%%%%%%%%%%%%%
TShoulder = gWorldS1fun([kinPars MkSuppDimensions], Joint_start);
DistWorldShoulder = TShoulder(1:3,4);
TElbow = gWorldE1fun([kinPars MkSuppDimensions], Joint_start);
DistWorldElbow = TElbow(1:3,4);
TElbow = gWorldE1fun([kinPars MkSuppDimensions], Joint_start);
DistWorldElbow = TElbow(1:3,4);
TWrist = gWorldW1fun([kinPars MkSuppDimensions], Joint_start);
DistWorldWrist = TWrist(1:3,4);
THand = gWorldHfun([kinPars MkSuppDimensions], Joint_start);
DistWorldHand = THand(1:3,4);

%%%%%%%%%%%%%%%% plot links %%%%%%%%%%%%%%%%%%%%%%%%
plot3([DistWorldShoulder(1) DistWorldElbow(1) DistWorldWrist(1) DistWorldHand(1)],...
      [DistWorldShoulder(2) DistWorldElbow(2) DistWorldWrist(2) DistWorldHand(2)],...
      [DistWorldShoulder(3) DistWorldElbow(3) DistWorldWrist(3) DistWorldHand(3)],'Color',[191/255 191/255 239/255],'LineWidth',10)

%%%%%%%%%%%%%%%% plot joints %%%%%%%%%%%%%%%%%%%%%%%%
plot3(DistWorldShoulder(1),DistWorldShoulder(2),DistWorldShoulder(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
plot3(DistWorldElbow(1),DistWorldElbow(2),DistWorldElbow(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
plot3(DistWorldWrist(1),DistWorldWrist(2),DistWorldWrist(3),'ok','MarkerSize',10,'MarkerFaceColor','k')

%%
%%%%%%%%%%%%%%%% Estimated Positions of Joints %%%%%%%%%%%%%%%%%%%%%%%%
TShoulder = gWorldS1fun([kinPars MkSuppDimensions], Joint_final);
DistWorldShoulder = TShoulder(1:3,4);
TElbow = gWorldE1fun([kinPars MkSuppDimensions], Joint_final);
DistWorldElbow = TElbow(1:3,4);
TElbow = gWorldE1fun([kinPars MkSuppDimensions], Joint_final);
DistWorldElbow = TElbow(1:3,4);
TWrist = gWorldW1fun([kinPars MkSuppDimensions], Joint_final);
DistWorldWrist = TWrist(1:3,4);
THand = gWorldHfun([kinPars MkSuppDimensions], Joint_final);
DistWorldHand = THand(1:3,4);

%%%%%%%%%%%%%%%% plot links %%%%%%%%%%%%%%%%%%%%%%%%
plot3([DistWorldShoulder(1) DistWorldElbow(1) DistWorldWrist(1) DistWorldHand(1)],...
      [DistWorldShoulder(2) DistWorldElbow(2) DistWorldWrist(2) DistWorldHand(2)],...
      [DistWorldShoulder(3) DistWorldElbow(3) DistWorldWrist(3) DistWorldHand(3)],'Color',[191/255 191/255 239/255],'LineWidth',10)

%%%%%%%%%%%%%%%% plot joints %%%%%%%%%%%%%%%%%%%%%%%%
plot3(DistWorldShoulder(1),DistWorldShoulder(2),DistWorldShoulder(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
plot3(DistWorldElbow(1),DistWorldElbow(2),DistWorldElbow(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
plot3(DistWorldWrist(1),DistWorldWrist(2),DistWorldWrist(3),'ok','MarkerSize',10,'MarkerFaceColor','k')

%%

%%%%%%%%%%%%%%%% plot body %%%%%%%%%%%%%%%%%%%%%%%%
x = [DistWorldShoulder(1) DistWorldShoulder(1)      DistWorldShoulder(1)     DistWorldShoulder(1)];
y = [DistWorldShoulder(2) DistWorldShoulder(2)-400  DistWorldShoulder(2)-400 DistWorldShoulder(2)];
z = [DistWorldShoulder(3) DistWorldShoulder(3)      DistWorldShoulder(3)-100 DistWorldShoulder(3)-100];
fill3(x,y,z,'yellow')

x = [DistWorldShoulder(1) DistWorldShoulder(1)-600  DistWorldShoulder(1)-600 DistWorldShoulder(1)];
y = [DistWorldShoulder(2) DistWorldShoulder(2)      DistWorldShoulder(2)     DistWorldShoulder(2)];
z = [DistWorldShoulder(3) DistWorldShoulder(3)      DistWorldShoulder(3)-100 DistWorldShoulder(3)-100];
fill3(x,y,z,'yellow')

x = [DistWorldShoulder(1) DistWorldShoulder(1)      DistWorldShoulder(1)-600 DistWorldShoulder(1)-600];
y = [DistWorldShoulder(2) DistWorldShoulder(2)-400  DistWorldShoulder(2)-400 DistWorldShoulder(2)];
z = [DistWorldShoulder(3) DistWorldShoulder(3)      DistWorldShoulder(3)     DistWorldShoulder(3)];
fill3(x,y,z,'yellow')

x_col = [255 153 153]/255;
y_col = [153 255 153]/255;
z_col = [153 153 255]/255;
coord_syst_colors = [x_col; y_col; z_col];
draw_coordinate_system(50,THand(1:3,1:3),THand(1:3,4),coord_syst_colors)

% 
% h = patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
%          'EdgeColor',       'none',        ...
%          'FaceLighting',    'gouraud',     ...
%          'AmbientStrength', 0.15);
% camlight('headlight');
% material('dull');

%rotate(h,[1 0 0],-90)
%rotate(h,[0 0 1],-90)

axis( [-500 800 -500 800 -500 800])

view ( 130.8000,10)
xlabel x; ylabel y; zlabel z;

end