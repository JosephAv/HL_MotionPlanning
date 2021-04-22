function [traiettoria, tempo, viapoint] = calcolo_traj_2(start, t_s, finish, t_f, obs, offset, fpc, v_start, a_start, side)
%In questa versione viene fatto il controllo sulla lunghezza della
%traiettoria su quella reale e non quella approssimata


l_opt = inf;
d_obs_vp = inf; %Aggiunta per ridurre il tempo di calcolo
k_obs = 1.5; %Aggiunta per ridurre il tempo di calcolo
traiettoria = [];
tempo = [];
N = 500;


if strcmp(side, 'r')
    ws_x_lim = [-200 500]; %Limiti dello spazio di lavoro ("cubico") in mm
    ws_y_lim = [-300 700];
    ws_z_lim = [0 600];
%     ws_x_lim = [400 500]; %Limiti dello spazio di lavoro test reali in mm
%     ws_y_lim = [-500 700];
%     ws_z_lim = [150 900];
else
    ws_x_lim = [-200 500]; %Limiti dello spazio di lavoro ("cubico") in mm
    ws_y_lim = [-700 300];
    ws_z_lim = [0 600];
end

for i=1:N
    x = [ws_x_lim(1)+(ws_x_lim(2)-ws_x_lim(1))*rand...
         ws_y_lim(1)+(ws_y_lim(2)-ws_y_lim(1))*rand...
         ws_z_lim(1)+(ws_z_lim(2)-ws_z_lim(1))*rand];
    if not(point_check_collision(x, obs, offset))
        d_x = norm(x-obs(1:3)); %Aggiunta per ridurre il tempo di calcolo
        if d_x<k_obs*d_obs_vp %Aggiunta per ridurre il tempo di calcolo
        l_dx = norm(start-x);
        l_sx = norm(x-finish);
        t_x = (l_dx/(l_dx+l_sx))*(t_f-t_s);
        cand_traj = [];
        t = [];
        for j=1:3
            [cand_traj_i, t_i] = traj_obs(fpc{j}, start(j), finish(j), v_start(j), a_start(j), t_f-t_s, [x(j) t_x]);
            cand_traj = [cand_traj cand_traj_i];
            t = [t, t_i+t_s ];
        end
        if not(traj_check_collision(cand_traj, obs, offset))
            l_tot = 0;
            for k=1:(length(cand_traj(:,1))-1)
                l_tot = l_tot + norm((cand_traj(k+1,:)-cand_traj(k,:)));
            end
            if l_tot < l_opt
                traiettoria = cand_traj;
                tempo = t;
                l_opt = l_tot;
                d_obs_vp = d_x; %Aggiunta per ridurre il tempo di calcolo
                viapoint = [x (t_x+t_s)];
            end
        end
        end
    end
end

if l_opt == inf
    disp('Planning Fallito')
end
end