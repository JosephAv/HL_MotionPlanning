function [ Ropt, dopt, T_hom, T_hom_inv] = OptimalRigidPose(XT, YT)

%This function calculate rotation and translation between global system to
%the one defined by chest support.
%XT are the local coordinates  of points P_i, i.e. in the cad model 
%YT are the global coordinates of points P_i, i.e. as measured by the Phase Space
%they are expected to be organized as 
%XT = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4]
X = XT';
Y = YT';

xbar = mean(XT)';
ybar = mean(YT)';

for i = 1:4
Xtilde(:,i) = X(:,i) - xbar;
Ytilde(:,i) = Y(:,i) - ybar;
end

Co = Ytilde*Xtilde';

[U, Sigma, V] = svd(Co);
NewSigma = [1 0 0; 0 1 0; 0 0 det(U*V')];


Ropt = U*NewSigma*V';		
dopt = ybar - Ropt*xbar;

T_hom = [Ropt dopt; [0 0 0 1]]; %rotation from local to global

T_hom_inv = [Ropt' -Ropt'*dopt; [0 0 0 1]]; %rotation from global to local T_hom_inv = inv(T_hom)
		
end

