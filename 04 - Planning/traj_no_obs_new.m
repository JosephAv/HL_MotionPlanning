function [traiettoria, t] = traj_no_obs_new(fpc,start,finish, t_f, v_start, v_finish)
%A differenza di traj_no_obs questo vincola anche velocità iniziale e
%finale della traiettoria

media = fpc(:,1);
pc1 = fpc(:,2);
pc2 = fpc(:,3);
pc3 = fpc(:,4);
l = length(media);
t_c = t_f/(l-1);

vm_s = (media(2) - media(1))/t_c;
vm_f = (media(l) - media(l-1))/t_c;

v1_s = (pc1(2) - pc1(1))/t_c;
v1_f = (pc1(l) - pc1(l-1))/t_c;

v2_s = (pc2(2) - pc2(1))/t_c;
v2_f = (pc2(l) - pc2(l-1))/t_c;

v3_s = (pc3(2) - pc3(1))/t_c;
v3_f = (pc3(l) - pc3(l-1))/t_c;

b = [start-media(1); finish-media(l); (v_start-vm_s); (v_finish-vm_f)];
    
A = [1           pc1(1)        pc2(1)        pc3(1);...
     1           pc1(l)        pc2(l)        pc3(l);...
     0             v1_s          v2_s          v3_s;...
     0             v1_f          v2_f          v3_f];...

A = inv(A);
x = A*b;

traiettoria = x(1) + media + x(2)*pc1 + x(3)*pc2 + x(4)*pc3;
t = (t_c*[0:l-1])';

end

