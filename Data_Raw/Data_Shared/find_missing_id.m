function [ missids ] = find_missing_id( IDnow,MkIDS )
%this function find which mk is missing in IDnow w.r.t MkIDS
missids = [];

    for i = 1 : length(MkIDS)
        aux = IDnow - MkIDS(i);
        index = find(~aux); %Find 0. If a 0 exist (only 1) then mk is present 
        
        if numel(index) == 0 %if mk is not present
            missids = [missids MkIDS(i)];
        end
        
    end

end

