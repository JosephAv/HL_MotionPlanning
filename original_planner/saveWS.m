current_time = clock; % get the date and the time
dir = num2str(current_time(1));
for i = 2:5
  dir = strcat(dir,'_',num2str(current_time(i))); 
end

name = strcat('data/WS_data_',dir,'.mat');

save(name);