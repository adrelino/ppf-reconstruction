function [] = global_graph_opt_easy()
close all;
axis vis3d;

n = 1000;
zNoise = 0.001;
yNoise = 0.25;

T = eye(4);
T(1:3,4)=[0.1,0.5,3];
T(1:3,1:3)=eulerAngle(pi/4,-pi*0.9,pi/2);

ptsRef = rand(n,3);
ptsRef(:,3) = ptsRef(:,3)*zNoise;
ptsRef(:,2) = ptsRef(:,2)*yNoise;

% ptsRef2 = ptsRef;
% ptsRef2(:,3) = ptsRef2(:,3)+0.8;

% noise = randn(n,3)*0.01;
% noise(:,3) = noise(:,3)*zNoise;
% noise(:,2) = noise(:,2)*yNoise;
% ptsPreProj = ptsPreProj+noise;

%pts = project(T,ptsPreProj);

T3 = eye(4);
T3(1:3,4)=[-0.1,-0.5,-3];
T3(1:3,1:3)=eulerAngle(-pi/4,pi*0.9,-pi/2);

clouds{1}=ptsRef;
clouds{2}=ptsRef;
clouds{3}=ptsRef;

poses{1}=eye(4);
poses{2}=T;
%poses{3}=T3;

for k=1:length(poses)
    plotCloud(project(poses{k},clouds{k}),'g');hold on;
    h(k)=plotCloud(project(poses{k},clouds{k}),'b');hold on;  
end

Corr=cell(3,3);
Corr{1,2}.src=clouds{1};
Corr{1,2}.dst=clouds{2};
Corr{1,3}.src=clouds{1};
Corr{1,3}.dst=clouds{3};

Corr{2,1}.src=clouds{2};
Corr{2,1}.dst=clouds{1};
Corr{2,3}.src=clouds{2};
Corr{2,3}.dst=clouds{3};

Corr{3,1}.src=clouds{3};
Corr{3,1}.dst=clouds{1};
Corr{3,2}.src=clouds{3};
Corr{3,2}.dst=clouds{2};

fixed=[0 0 0];


%Ref = lm_icp_step_twistsSimple(pts,ptsRef);
mv_lm_icp_step(Corr,poses,h,clouds,fixed);
    
end

function [ D ] = project(P,C)
    Cok=[C(:,1:3) ones(size(C,1),1)]';
    Dbad=(P*Cok)';
    D=Dbad(:,1:3);
end

function [h] = plotCloud(cloud,color)
   h=plot3(cloud(:,1),cloud(:,2),cloud(:,3),'.','Color',color);
   hold on;
end