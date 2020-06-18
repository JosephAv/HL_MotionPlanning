close all
clear all
clc

%%
tic
[pos, joints, acc, rec_date]=  import_mvnx('P08_SoftPro_gestures.mvnx')
%E:\
%E:\Anne_test
%[pos, joints, acc, rec_date]=  import_mvnx('E:\xsens\test-001_beta_SG-Abd_handtouch.mvnx')



%TREE = load_mvnx('P08_SoftPro_gestures.mvnx')

%filename = ('P10_SoftPro_grasp+manips_0.mvnx')

%main_mvnx

toc

plot(joints.jLeftShoulder_y)

%%

filename = ('P08_SoftPro_gestures.mvnx')

main_mvnx
%%

[pos, joints, acc, rec_date]=  import_mvnx('P08_SoftPro_gestures.mvnx')


2018 Xsens Version Analysis

filename = ('E:\OT-Case-Analysis\FMA-Flex_r3')
main_mvnx

dat = [];
for i =4:length(tree.subject.frames.frame  )
   dat = [dat; tree.subject.frames.frame(i).jointAngleXZY(22)]; 

tree = load_mvnx('E:\OT-Case-Analysis\FMA-Flex_r3')

dat = [];
for i =4:length(tree.subject.frames.frame  )
   dat = [dat; tree.subject.frames.frame(i).jointAngleXZY(22)]; 
end
plot (dat)
hold on
dat = [];
for i =4:length(tree.subject.frames.frame  )
   dat = [dat; tree.subject.frames.frame(i).jointAngleXZY(23)]; 
end
plot (dat)
dat = [];
for i =4:length(tree.subject.frames.frame  )
   dat = [dat; tree.subject.frames.frame(i).jointAngleXZY(24)]; 
end
plot (dat)
dat = [];
for i =4:length(tree.subject.frames.frame  )
   dat = [dat; tree.subject.frames.frame(i).jointAngle(27)]; 
end
plot (dat)
end