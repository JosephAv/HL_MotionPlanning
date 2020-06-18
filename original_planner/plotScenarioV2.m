function plotScenarioV2(kinPars, P_start, Joint_start, P_final, Joint_final, Obs)
%this function plots the scenario, with initial configuration and list of
%obstacles



if nargin < 6
    Obs = [];
elseif size(Obs,1) == 3
    Obs = Obs';
end

x_col = [255 153 153]/255;
y_col = [153 255 153]/255;
z_col = [153 153 255]/255;
coord_syst_colors = [x_col; y_col; z_col];

%% Visualization UP

AnglesNow{1} = [0 0 0 0 0 0 0]';

TABS_UP{1} = fkUP(AnglesNow{1},kinPars);

[x, y, z] = sphere();
Joint_surfUP = surf(30*x, 30*y, 30*z);
jointcolors = jet(3);
for k = 1:3
    join_sph{k} = patch(surf2patch(Joint_surfUP));
    set(join_sph{k}, 'facec', jointcolors(k,:)); 
    set(join_sph{k}, 'EdgeColor','none');       % Set the edge color

    if ismember(k, [2,3]) 
        p1 = TABS_UP{1}{k-1}(1:3,4);
        p2 = TABS_UP{1}{k}(1:3,4);
        hlUP{k} = line([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'LineWidth',4);
    end
    t_join_sph{k} = hgtransform('Parent',gca);
    set(t_join_sph{k},'Matrix',[[eye(3),TABS_UP{1}{k}(1:3,4)];0 0 0 1])
    set(join_sph{k},'Parent',t_join_sph{k});
end
VisualTHand = eye(4);
load handCADUP
p_handUP = patch('faces', CAD_F_s_UP, 'vertices' ,CAD_V_s_UP);
t_handUP = hgtransform('Parent',gca);
set(t_handUP,'Matrix',TABS_UP{1}{4});
set(p_handUP,'Parent',t_handUP);
set(p_handUP, 'facec', 'y');             % Set the face color (force it)
set(p_handUP, 'EdgeColor','k');       % Set the edge color
delete(Joint_surfUP);
daspect([1 1 1]) % Setting the aspect ratio
xlabel('X'),ylabel('Y'),zlabel('Z')
axis equal
view( -173.0200,21.36);
xlim([-1000 1000])
ylim([-700 700])
zlim([-700 700])

%% Torso
load torsoCAD
CAD_V_s_TORSO(:,2) = CAD_V_s_TORSO(:,2)-100;
p_torso = patch('faces', CAD_F_s_TORSO, 'vertices' ,CAD_V_s_TORSO);
set(p_torso, 'facec', 'b');             % Set the face color (force it)
set(p_torso, 'EdgeColor','k');       % Set the edge color

%set(gcf,'Position',[-1919 -134.2000 1920 930.4000])

%% plot coordinate system
TABS_UP_start{1} = fkUP(Joint_start,kinPars);
draw_coordinate_system(50,TABS_UP_start{1}{4}(1:3,1:3),TABS_UP_start{1}{4}(1:3,4),coord_syst_colors)

TABS_UP_end{1} = fkUP(Joint_final,kinPars);
draw_coordinate_system(50,TABS_UP_end{1}{4}(1:3,1:3),TABS_UP_end{1}{4}(1:3,4),coord_syst_colors)


%% plot head
load headCAD
p_head =  patch('faces', CAD_F_head, 'vertices' ,CAD_V_head+[-100 -100 0]);
t_head = hgtransform('Parent',gca);
set(t_head,'Matrix',eye(4));
set(p_head,'Parent',t_head);
set(p_head, 'facec', 'y');             % Set the face color (force it)
set(p_head, 'EdgeColor',[255,165,0]./255);       % Set the edge color

%% plot obstacles
if ~isempty(Obs)
    PreT = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
    Obs = (PreT(1:3,1:3)*(Obs'))';
    plot3(Obs(:,1),Obs(:,2),Obs(:,3),'*','Color',[255/255 0/255 0/255],'MarkerSize',4)
end



%% Update pose based on movement from the task

view([-160,50])
axis equal
xlim([-700 700])
ylim([-700 700])
zlim([-700 700])
set(gcf,'Position',[88.2000   46.6000  965.6000  715.2000])


%plot arm
AnglesNow =  Joint_start;
TABS_UP{1} = fkUP(AnglesNow,kinPars);
for k = 1:3
    if ismember(k, [2,3])
        p1 = TABS_UP{1}{k-1}(1:3,4);
        p2 = TABS_UP{1}{k}(1:3,4);
        hlUP{k}.XData = [p1(1) p2(1)];
        hlUP{k}.YData = [p1(2) p2(2)];
        hlUP{k}.ZData = [p1(3) p2(3)];
    end
    set(t_join_sph{k},'Matrix',[[eye(3),TABS_UP{1}{k}(1:3,4)];0 0 0 1])
end
set(t_handUP,'Matrix',TABS_UP{1}{4}*VisualTHand);
drawnow



%title(['Synergy number ' num2str(synnum) ' Coefficient value: ' num2str(coeffs(i))])

end
%% Forward kinematic function

function [] = deletecoordsys(h)
    delete(h.line.x)
    delete(h.line.y)
    delete(h.line.z)
    delete(h.cone.x)
    delete(h.cone.y)
    delete(h.cone.z)
    delete(h.text.x)
    delete(h.text.y)
    delete(h.text.z)
end

function [TABS] = fkUP(AnglesNow, kinPars)
    %%%%%%%%%%%%%%%%%%%%%% Reference Sys Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PreT = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
    %%%%%%%%%%%%%%%% Estimated Positions of Joints %%%%%%%%%%%%%%%%%%%%%%%%
    UpperLimbParametersDEF = kinPars;
    TShoulder = gWorldS1fun([UpperLimbParametersDEF ], AnglesNow);
    %DistWorldShoulder = TShoulder(1:3,4);
    TElbow = gWorldE1fun([UpperLimbParametersDEF ], AnglesNow);
    %DistWorldElbow = TElbow(1:3,4);
    TWrist = gWorldW1fun([UpperLimbParametersDEF ], AnglesNow);
    %DistWorldWrist = TWrist(1:3,4);
    THand = gWorldHfun([UpperLimbParametersDEF ], AnglesNow);
    %DistWorldHand = THand(1:3,4);
    
    TABS{1} = PreT*TShoulder;
    TABS{2} = PreT*TElbow;
    TABS{3} = PreT*TWrist;
    TABS{4} = PreT*THand;
 
end