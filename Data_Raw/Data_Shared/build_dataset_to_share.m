close all
clear all
clc

% create new folders
for i = 1 : 42
    mkdir(['DATA_to_share/S' num2str(i)])
end

% files = dir;
names = {'11_Antonio_28', '12_davide_25', '13_carlotta_26', '14_Chiara_25',...
    '15_cristiano_32', '16_daniele_26', '17_Dario_25', '18_emanuele_27',...
    '19_gemma_24', '1_vincenzo_25', '20_gianluca_27', '21_giordano_27',...
    '22_Grazia_24', '23_Laura_28', '24_miriam_24', '25_Roberta_26',...
    '26_rosa_24', '27_Rossana_23', '28_Sara_21', '29_silvia_26',...
    '2_simone_23', '30_simone_27', '31_joana_26', '32_Costanza_29',...
    '33_jose_33', '34_Licia_26', '35_Mimma_30', '36_silvia_25',...
    '37_andrea_33', '38_chiara_24', '39_federica_30', '3_federica_30',...
    '40_giuseppe_25', '41_pinuccia_25', '42_sonia_27', '4_nicoletta_27',...
    '5_franca_21', '6_Antonio_27', '7_Michele_27', '8_marco_22', '9_Anna_23', '10_Riccardo_26'};

names_file = {'Antonio_28', 'davide_25', 'carlotta_26', 'Chiara_25',...
    'cristiano_32', 'daniele_26', 'Dario_25', 'emanuele_27',...
    'gemma_24', 'vincenzo_25', 'gianluca_27', 'giordano_27',...
    'Grazia_24', 'Laura_28', 'miriam_24', 'Roberta_26',...
    'rosa_24', 'Rossana23', 'Sara_21', 'silvia_26',...
    'simone_23', 'simone_27', 'joana_26', 'Costanza_29',...
    'josä_33', 'Licia_26', 'Mimma_30', 'Silva25',...
    'andrea_33', 'chiara_24', 'federica_30', 'federica_30',...
    'giuseppe_25', 'pinuccia_25', 'sonia_27', 'Nicoletta_27',...
    'franca_21', 'Antonio_27', 'Michele_27', 'marco_22', 'Anna_23', 'riccardo_26'};

% files = dir;
names_to_share = {'S11', 'S12', 'S13', 'S14',...
    'S15', 'S16', 'S17', 'S18',...
    'S19', 'S1', 'S20', 'S21',...
    'S22', 'S23', 'S24', 'S25',...
    'S26', 'S27', 'S28', 'S29',...
    'S2', 'S30', 'S31', 'S32',...
    'S33', 'S34', 'S35', 'S36',...
    'S37', 'S38', 'S39', 'S3',...
    'S40', 'S41', 'S42', 'S4',...
    'S5', 'S6', 'S7', 'S8', 'S9', 'S10'};


working_folder = cd;
saving_folder = 'DATA_to_share/';

for subjno =  1 : numel(names)
    disp(['working on subj ' num2str(subjno) ' of ' num2str( numel(names))])
    folder_name = names{subjno};
    mytab(subjno).name = names{subjno};

    for taskno = 1 : 30
        for repno = 1 : 7
            
            Subject = names_file{subjno};

            %Filename = 'task7.dat';

            Task = ['_' num2str(taskno)];
            Repetition = ['_' num2str(repno)];
            Filename = [Subject Task Repetition];
            
            
            try 
                data = load([folder_name '/' Filename '.dat']);
            catch
%                 disp([[folder_name '/' Filename '.dat'] ' not found'])
                continue
            end
                cd(['..\SUBJ' names{subjno} ''])
                IDS_Definition
                cd(working_folder)

                repack_data

                numframes = length(data1);
                
                name_evt = ['evt' num2str(taskno) '_' num2str(repno)];
                eval(['mytab(subjno).',name_evt,' = numframes/100;'])
                
                subfolder = names_to_share{subjno};
                saving_folder_complete =  strcat(saving_folder, subfolder, '/',  names_to_share{subjno}, Task, Repetition );
                data=data1;
                save(saving_folder_complete,'data')
 
        end
    end
end
