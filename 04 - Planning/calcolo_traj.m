function [traiettoria, tempo, viapoint] = calcolo_traj(start, t_s, finish, t_f, obs, offset, fpc, v_start, a_start)

l_opt = inf;
traiettoria = [];
tempo = [];
N = 10000;


    ws_x_lim = [-200 500]; %Limiti dello spazio di lavoro ("cubico") in mm
    ws_y_lim = [-300 700];
    ws_z_lim = [0 600];
%     ws_x_lim = [400 500]; %Limiti dello spazio di lavoro test reali in mm
%     ws_y_lim = [-500 700];
%     ws_z_lim = [150 900];


for i=1:N
    x = [ws_x_lim(1)+(ws_x_lim(2)-ws_x_lim(1))*rand...
         ws_y_lim(1)+(ws_y_lim(2)-ws_y_lim(1))*rand...
         ws_z_lim(1)+(ws_z_lim(2)-ws_z_lim(1))*rand];
    if not(point_check_collision(x, obs, offset))
        l_dx = norm(start-x);
        l_sx = norm(x-finish);
        l_tot = l_dx + l_sx;
        t_x = (l_dx/l_tot)*(t_f-t_s) + t_s;
        if l_tot < l_opt
            cand_traj = [];
            t = [];
            for j=1:3
                [cand_traj_i, t_i] = traj_obs(fpc{j}, start(j), finish(j), v_start(j), a_start(j), t_f-t_s, [x(j) t_x]);
                cand_traj = [cand_traj cand_traj_i];
                t = [t, t_i+t_s ];
            end
            if not(traj_check_collision(cand_traj, obs, offset))
                traiettoria = cand_traj;
                tempo = t;
                l_opt = l_tot;
                viapoint = [x (t_x+t_s)];
            end
        end
    end
end

if l_opt == inf
    disp('Planning Fallito')
end
end