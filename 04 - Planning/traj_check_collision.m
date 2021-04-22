function [collisione] = traj_check_collision(traj, obs, offset)
%Controlla se ogni punto di una determinata traiettoria traj si trova
%esternamente ad ogni ostacolo contenuto in obs di almeno una distanza pari a offset
%obs è una matrice in cui vengono impilati per righe una serie di ostacoli
%sferici
%Ogni ostacolo sferico è definito da una riga di obs con la seguente
%struttura: [centro, raggio]
%traj contiene tutti i campioni della traiettoria impilati per righe

n_sample = length(traj(:,1));
collisione = false;

for i=1:2:n_sample
    if point_check_collision(traj(i,:), obs, offset)
        collisione = true;
        return
    end
end

end