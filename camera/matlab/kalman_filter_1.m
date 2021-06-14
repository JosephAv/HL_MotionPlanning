function [Pxf,Pyf,Pzf,Vxf,Vyf,Vzf] = kalman_filter_1(Px,Py,Pz,Vx,Vy,Vz,Delta_T,R,q)

[N,~]= size(Px);
C = eye(6);
%D = [1; 1; 1; 0; 0; 0];

% predizione iniziale
A = [eye(3) eye(3)*Delta_T(1);zeros(3,3), eye(3)];
xk = [Px(1); Py(1); Pz(1); Vx(1); Vy(1); Vz(1)];
xk_1k = A*xk;

Pk_1k = R;

x_filtrato = zeros(N,6);
x_filtrato(1,:) = xk;

for i=2:1:N
    
    D = [xk_1k(4); xk_1k(5); xk_1k(6); 0; 0; 0];
    % correzione
    xkk_1 = xk_1k;
    Pkk_1 = Pk_1k;
    yk = [Px(i); Py(i); Pz(i); Vx(i); Vy(i); Vz(i)];
    ek = yk - C*xkk_1;
    Sk = R + C*Pkk_1*C';
    Lk = Pkk_1*C'*inv(Sk);
    
    xkk = xkk_1 + Lk*ek;
    Pkk = (eye(6) - Lk*C)*Pkk_1*(eye(6)-Lk*C)' + Lk*R*Lk';
    
    x_filtrato(i,:) = xkk;
    
    % predizione
    Ak = [eye(3) eye(3)*Delta_T(i);zeros(3,3), eye(3)];
    
    xk_1k = Ak*xkk;
    Pk_1k = Ak*Pkk*Ak' + D*q*D';
    
end

Pxf = x_filtrato(:,1);
Pyf = x_filtrato(:,2);
Pzf = x_filtrato(:,3);
Vxf = x_filtrato(:,4);
Vyf = x_filtrato(:,5);
Vzf = x_filtrato(:,6);

end