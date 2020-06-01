%% funzione per il calcolo di H
%Calcolo della matrice H da utilizzare nell'EKF per un singolo marker.
%StarLocalCoords sono le coordinate nel sistema di riferimento locale del
%singolo marker
function H = calcolo_H(x_stim, StarLocalCoords)

dx = x_stim(1);
dy = x_stim(2);
dz = x_stim(3);
gamma = x_stim(4);
beta = x_stim(5);
alfa = x_stim(6);

p1x = StarLocalCoords(1,1);
p1y = StarLocalCoords(1,2);
p1z = StarLocalCoords(1,3);

H =     [ 1, 0, 0,   p1y*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) + p1z*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)), p1z*cos(alfa)*cos(beta)*cos(gamma) - p1x*cos(alfa)*sin(beta) + p1y*cos(alfa)*cos(beta)*sin(gamma), p1z*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) - p1y*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)) - p1x*cos(beta)*sin(alfa); ...
          0, 1, 0, - p1y*(cos(alfa)*sin(gamma) - cos(gamma)*sin(alfa)*sin(beta)) - p1z*(cos(alfa)*cos(gamma) + sin(alfa)*sin(beta)*sin(gamma)), p1z*cos(beta)*cos(gamma)*sin(alfa) - p1x*sin(alfa)*sin(beta) + p1y*cos(beta)*sin(alfa)*sin(gamma), p1z*(sin(alfa)*sin(gamma) + cos(alfa)*cos(gamma)*sin(beta)) - p1y*(cos(gamma)*sin(alfa) - cos(alfa)*sin(beta)*sin(gamma)) + p1x*cos(alfa)*cos(beta); ...
          0, 0, 1,                                                                         p1y*cos(beta)*cos(gamma) - p1z*cos(beta)*sin(gamma),                             - p1x*cos(beta) - p1z*cos(gamma)*sin(beta) - p1y*sin(beta)*sin(gamma),                                                                                                                                                   0];