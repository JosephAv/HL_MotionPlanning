function [collisione] = point_check_collision(p, obs, offset)
%Controlla se un punto p si trova esternamente al set di ostacoli obs di
%una distanza almeno pari a offset. 
%p � un vettore riga
%obs � una matrice in cui vengono impilati per righe una serie di ostacoli
%sferici
%Ogni ostacolo sferico � definito da una riga di obs con la seguente
%struttura: [centro, raggio]

collisione = false;
n_obs = length(obs(:,1));

for i=1:n_obs
    if norm(p-obs(i,1:3))<(obs(i,4)+offset)
        collisione = true;
        return
    end
end

end