function plotTrajectoryV2(Traj_act,kinPars, P_target, Joint_target, P_start, Joint_start, Obs, num_syns_used)
    
if nargin < 6
    Obs = [];
elseif 0 %size(Obs,1) == 7
    Obs = Obs';
end

x_col = [255 153 153]/255;
y_col = [153 255 153]/255;
z_col = [153 153 255]/255;
coord_syst_colors = [x_col; y_col; z_col];
velocity = diff(Traj_act); 
velocity = [velocity; velocity(end,:)]; %add a sample to avoid missing last element in the plotting for

current_time = clock; % get date/time
dir = num2str(current_time(1));
for i = 2:5
  dir = strcat(dir,'_',num2str(current_time(i))); 
end

namevideo = strcat('Videos/Reconstructed_',num2str(num_syns_used),'syns_', dir);
v = VideoWriter([namevideo,'.avi']);
open(v);
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
view([-160,50]);
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

%plot coordinate system
TABS_UP_start{1} = fkUP(Joint_start,kinPars);
draw_coordinate_system(50,TABS_UP_start{1}{4}(1:3,1:3),TABS_UP_start{1}{4}(1:3,4),coord_syst_colors)

TABS_UP_end{1} = fkUP(Joint_target,kinPars);
draw_coordinate_system(50,TABS_UP_end{1}{4}(1:3,1:3),TABS_UP_end{1}{4}(1:3,4),coord_syst_colors)


% plot head
load headCAD
p_head =  patch('faces', CAD_F_head, 'vertices' ,CAD_V_head+[-100 -100 0]);
t_head = hgtransform('Parent',gca);
set(t_head,'Matrix',eye(4));
set(p_head,'Parent',t_head);
set(p_head, 'facec', 'y');             % Set the face color (force it)
set(p_head, 'EdgeColor',[255,165,0]./255);       % Set the edge color

% plot obstacles
if ~isempty(Obs)
    PreT = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
    Obs = (PreT(1:3,1:3)*(Obs'))';
    plot3(Obs(:,1),Obs(:,2),Obs(:,3),'*','Color',[255/255 0/255 0/255],'MarkerSize',4)
end

%% Update pose based on movement from the task

%%%%%%%%%%%%%%%%%%%%% views %%%%%%%%%%%%%%%%%%%%%%%%
view([-160,50])% view([-107,13]) %normal view
% view([-158,-2])% view([-107,13]) %other view for t1 obs1
% view([-100,13])% view([-107,13]) %other view for t2 obs1
%%%%%%%%%%%%%%%%%%%%% views %%%%%%%%%%%%%%%%%%%%%%%%
azs = linspace(-250,-130,size(Traj_act,1)-1);

axis equal
xlim([-500 700])
ylim([-700 500])
zlim([-700 700])
% set(gcf,'Position',[88.2000   46.6000  965.6000  715.2000])
%  set(gca, 'Color', 'None'), 
%  grid off,
 box off,
 
 %define 3 different axes for plot components
 cf = gcf;
 set(gcf,'color','w');
 ax1 = cf.CurrentAxes;
 ax1.Position = ax1.Position + [-0.18 0.0 0 0];
 ax2 = axes('position',[0.73,0.55,0.23,0.3]) ;
 ax3 = axes('position',[0.73,0.15,0.23,0.3]) ;
%  uistack(ax1,'bottom');


for time = 2 : size(Traj_act,1)
    %plot arm   
    axes(ax1)
%     view([azs(time-1),20]);

    AnglesNow =  Traj_act(time,:);
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
    draw_coordinate_system(50,TABS_UP{1}{4}(1:3,1:3),TABS_UP{1}{4}(1:3,4),coord_syst_colors)
%     title(['Number of Synergies used: ' num2str(num_syns_used) ' Time: ' num2str(time)])
    
%     axes('pos',[.6 .6 .2 .2])
%     plot(ones(1,10))

% 	set(ha,'handlevisibility','off', 'visible','off')

%%% plot joints
    axes(ax2)
    plot(Traj_act(1:time,:),'LineWidth',1)
%     xlabel('Time Cycle')
    xlim_val = size(Traj_act,1);
    xlim([0 xlim_val])
    xticks([0 floor(xlim_val/4) floor(xlim_val/2) floor(xlim_val/4*3) xlim_val])
    xticklabels({'0','25','50','75','100'})
    ylabel('Angle [rad]')
    ylim([-2 2.5])
    yticks([ -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5])
    grid on

%%% plot velocities
    axes(ax3)
    plot(velocity(1:time,:),'LineWidth',1)
    xlabel('Time Cycle')
    xlim_val = size(velocity,1);
    xlim([0 xlim_val])
    xticks([0 floor(xlim_val/4) floor(xlim_val/2) floor(xlim_val/4*3) xlim_val])
    xticklabels({'0','25','50','75','100'})
    ylabel('Joint Velocity [rad/ut]')    
    ylim([-0.05 0.05])    
    yticks([-0.05 -0.025 0 0.025 0.05])
    grid on

    drawnow

    frame = getframe(gcf);
    writeVideo(v,frame);  
end
close (v);

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