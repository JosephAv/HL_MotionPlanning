%% funzione per il calcolo di h
%Calcolo della funzione h da usare nelll'EKF per un singolo marker.
%StarLocalCoords sono le coordinate locali del singolo marker

function h = h_fun(x_stim, StarLocalCoords)

dx = x_stim(1);
dy = x_stim(2);
dz = x_stim(3);
gamma = x_stim(4);
beta = x_stim(5);
alfa = x_stim(6);

p1x = StarLocalCoords(1,1);
p1y = StarLocalCoords(1,2);
p1z = StarLocalCoords(1,3);

h =      [dx - p1y*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) + p1z*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) + p1x*cos(alfa)*cos(beta); ...
          dy + p1y*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - p1z*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) + p1x*cos(beta)*sin(alfa); ...
                                                                                          dz - p1x*sin(beta) + p1z*cos(beta)*cos(gamma) + p1y*cos(beta)*sin(gamma)];