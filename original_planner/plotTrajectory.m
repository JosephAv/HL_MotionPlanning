function plotTrajectory(Traj_act,kinPars, P_target, P_start, Joint_start, Obs)
    
if nargin < 6
    Obs = [];
elseif 0 %size(Obs,1) == 7
    Obs = Obs';
end


    
    
        addpath fPCA_scripts/kinematics

        MkSuppDimensions = [15 7 22.5 55]; 

        [NumFrame,~] = size(Traj_act); 
     
        v = VideoWriter(['Videos/Reconstructed.avi']);
        open(v);
        %figure;
        figure('units','normalized','outerposition',[0 0 1 1])


        for i = 1:NumFrame
            clf;hold on;
            x_col = [255 0 0]/255;
            y_col = [0 255 0]/255;
            z_col = [0 0 255]/255;
            coord_syst_colors_target = [x_col; y_col; z_col];
            
            P_start = calculatePosefromAnglesTraj(Joint_start,kinPars);
            draw_coordinate_system(50,quat2rotm(P_target(4:7)),P_target(1:3),coord_syst_colors_target) %draw target
            draw_coordinate_system(50,quat2rotm(P_start(4:7)),P_start(1:3),coord_syst_colors_target) %draw target
            
%             for ind = 1 : size(Obs,1)
%                 Pose = calculatePosefromAnglesTraj(Obs(ind,:),kinPars);
%                 draw_coordinate_system(50,quat2rotm(Pose(4:7)),Pose(1:3),coord_syst_colors_target) %draw target
%             end
            plot3(Obs(:,1),Obs(:,2),Obs(:,3),'*','Color',[255/255 0/255 0/255],'MarkerSize',4)

            
            AnglesNow = Traj_act(i,:);
            EstimationsNow = hfun([kinPars MkSuppDimensions], AnglesNow); %column vector 36 elem

            %%%%%%%%%%%%%%%%%%%%%% Reference Sys Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            plot3([0 30],[0 0],[0 0],'r','LineWidth',2)
            plot3([0 0],[0 30],[0 0],'g','LineWidth',2)
            plot3([0 0],[0 0],[0 30],'b','LineWidth',2)

            %%%%%%%%%%%%%%%% Estimated Positions of Joints %%%%%%%%%%%%%%%%%%%%%%%%
            TShoulder = gWorldS1fun([kinPars MkSuppDimensions], AnglesNow);
            DistWorldShoulder = TShoulder(1:3,4);
            TElbow = gWorldE1fun([kinPars MkSuppDimensions], AnglesNow);
            DistWorldElbow = TElbow(1:3,4);
            TElbow = gWorldE1fun([kinPars MkSuppDimensions], AnglesNow);
            DistWorldElbow = TElbow(1:3,4);
            TWrist = gWorldW1fun([kinPars MkSuppDimensions], AnglesNow);
            DistWorldWrist = TWrist(1:3,4);
            THand = gWorldHfun([kinPars MkSuppDimensions], AnglesNow);
            DistWorldHand = THand(1:3,4);

            %%%%%%%%%%%%%%%% plot links %%%%%%%%%%%%%%%%%%%%%%%%
            plot3([DistWorldShoulder(1) DistWorldElbow(1) DistWorldWrist(1) DistWorldHand(1)],...
                  [DistWorldShoulder(2) DistWorldElbow(2) DistWorldWrist(2) DistWorldHand(2)],...
                  [DistWorldShoulder(3) DistWorldElbow(3) DistWorldWrist(3) DistWorldHand(3)],'Color',[191/255 191/255 239/255],'LineWidth',10)

            %%%%%%%%%%%%%%%% plot joints %%%%%%%%%%%%%%%%%%%%%%%%
            plot3(DistWorldShoulder(1),DistWorldShoulder(2),DistWorldShoulder(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
            plot3(DistWorldElbow(1),DistWorldElbow(2),DistWorldElbow(3),'ok','MarkerSize',10,'MarkerFaceColor','k')
            plot3(DistWorldWrist(1),DistWorldWrist(2),DistWorldWrist(3),'ok','MarkerSize',10,'MarkerFaceColor','k')

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

            %view ( 130.8000,10) 
            view (-12.8000, 82.0000) %% good
            view(99.2000, 22.0000)
            xlabel x; ylabel y; zlabel z;

            frame = getframe(gca);
            writeVideo(v,frame);  
        end
        close (v);
end