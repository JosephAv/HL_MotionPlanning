function [Joint_des,Joint_start] = setRobotPoses(task)
%set initial and final poses from a string 

if strcmp(task,'ok')
    Joint_start = [0.0915 -0.7921 -0.6216 0.6544 -1.5007 0.0566 0.7626]; %EstimatedQ(:,100) licia 1_1
    Joint_des = [0.6536 -0.4006 0.0704 0.7927 -1.3243 0.8680 0.6280]; %EstimatedQ(:,500) licia 1_1
elseif strcmp(task,'exultation')
    Joint_start = [-0.4760 -0.0689 -0.7071 0.0306 -0.8596 0.5186 0.4707];
    Joint_des = [-1.6739 -0.6816 -0.3300 -0.2128 -0.0866 -0.2584 0.5165];   
elseif strcmp(task,'drink')
    Joint_start = [-0.2842 -0.8566 -0.9243 0.8557 -1.5240 -0.1247 0.7672];
    Joint_des = [0.6548 -0.7854 -0.1239 2.1830 -1.5175 0.5726 0.5893];  
elseif strcmp(task,'putonleft')
    Joint_start = [-0.3076 -0.5806 -0.8034 0.4744 -1.2319 0.2226 0.6541];
    Joint_des = [-0.3251 0.3432 -0.3933 0.4366 -0.6909 0.3593 0.1517];
elseif strcmp(task,'aaa')
elseif strcmp(task,'aaa')
elseif strcmp(task,'aaa')
elseif strcmp(task,'aaa')
elseif strcmp(task,'aaa')
elseif strcmp(task,'aaa')

else
    disp('no pose recognized')
    Joint_start = zeros(1,7); 
    Joint_des = zeros(1,7); 
end

end

