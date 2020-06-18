close all
clear all
clc

obs = rand(7,10);
etas = 0.3*ones(1,10);

syms q1 q2 q3 q4 q5 q6 q7 q
q = [q1; q2; q3; q4; q5; q6; q7];


q1n = -1:0.1:1;
q2n = -1:0.1:1; 
q3n = -1:0.1:1; 
q4n = -1:0.1:1; 
q5n = -1:0.1:1; 
q6n = -1:0.1:1; 
q7n = -1:0.1:1;




for j = 1 : size(obs,1)
    Pot(j,1) = 1/2*etas(1)/ (q(j)-obs(j,1))^2 ;
end
for i = 2 : size(obs,2)
    for j = 1 : size(obs,1)
        Pot(j,1) = Pot(j,1) + 1/2*etas(i)/ (q(j)-obs(j,i))^2 ;
    end
end

Pot_n = double(subs(Pot,q, zeros(7,1)))




figure, plot3(t1(1,:),t1(2,:),t1(3,:),'*'),hold on, plot3(Poly1(1,:),Poly1(2,:),Poly1(3,:),'r*')

plot3([P_traj(1) P_Poly(1)],[P_traj(2) P_Poly(2)],[P_traj(3) P_Poly(3)],'mo--')
