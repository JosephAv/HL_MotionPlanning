function [traiettoria, t] = traj_no_obs(fpc,start,finish, t_f)

media = fpc(:,1);
pc1 = fpc(:,2);
l = length(media);
t_c = t_f/(l-1);

b = [start - media(1); finish-media(end)];

A = [1 pc1(1); 1 pc1(end)];
x = inv(A)*b;

traiettoria = x(1) + media + x(2)*pc1;
t = (t_c*[0:l-1])';

end

