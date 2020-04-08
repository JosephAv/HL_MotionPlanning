
[ndata, ~] = size(data);

% Change them
% MkNUMBER = length(MkALL); 
MkNUMBER = 40;

frame = 1;
data2 = [];
data1 = cell(1,1);
data2old = 0;
for i = 1:ndata
    
    %here i create the cell structure
    if data(i,5) ~= 0
        data2 = [data2; data(i,[1 3:5])];
    elseif data(i,1) ~= 0
        frame = frame + 1;
        [n1, n2] = size(data2);
        
        %here i replace missing mks with 0s (in order to be compatible with Visar's code)
        if n1 ~= MkNUMBER
            IDnow = data2(:,1);
            missids = find_missing_id(IDnow,MkALL); %this function give missing ids

            for l = 1 : length(missids)
                aux = MkALL - missids(l);
                index = find(~aux); %Find the position of missing mk in MkIDS
                data2 = [data2(1:index-1,:); [missids(l) zeros(1,3)]; data2(index:end,:)];
                %data2 = [data2; [missids(l) zeros(1,3)]];
            end
            
            [nr, ~] = size(data2);
            for m = 1 : nr 
                if data2(m,2) == 0  && data2old(1,1)~=0
                    data2(m,2:4) = data2old(m,2:4);
                end
            end     
            
        end
        
        [n1, n2] = size(data2);
        data1 = [data1 mat2cell(data2,n1,n2)];    
        data2old = data2;
        data2 = [];
    end 

    %[mknr, mknc] = size(data2);
    
end

data1 = data1(2:end);
frame = frame-1;
clear data2