function [] = mv_lm_icp_easy()
close all;
axis vis3d;

n = 300;
zNoise = 0.001;
yNoise = 0.25;

sigma = 0.1;

T = eye(4);
T(1:3,4)=[0.1,0.5,3];
T(1:3,1:3)=eulerAngle(pi/4,-pi*0.9,pi/2);

ptsRef = rand(n,3);
ptsRef(:,3) = ptsRef(:,3)*zNoise;
ptsRef(:,2) = ptsRef(:,2)*yNoise;

noise = randn(n,3)*sigma;
noise(:,3) = noise(:,3)*zNoise;
noise(:,2) = noise(:,2)*yNoise;

noise2 = randn(n,3)*sigma;
noise2(:,3) = noise2(:,3)*zNoise;
noise2(:,2) = noise2(:,2)*yNoise;

T3 = eye(4);
T3(1:3,4)=[-0.1,-0.5,-3];
T3(1:3,1:3)=eulerAngle(-pi/4,pi*0.9,-pi/2);

frames{1}.pts=ptsRef;
frames{2}.pts=ptsRef+noise;
frames{3}.pts=ptsRef+noise2;

frames{1}.pose=eye(4);
frames{2}.pose=T;
frames{3}.pose=T3;

for k=1:length(frames)
    bla=project(frames{k}.pose,frames{k}.pts);
    center=mean(bla,1);
    text(center(1),center(2),center(3),num2str(k),'Color','r'); hold on;
    plotCloud(bla,'g');hold on;
    frames{k}.h=plotCloud(bla,'b');hold on;  
end

Corr=cell(3,3);

Corr{1,2}.src=frames{1}.pts;%(1:n*(2/3),:);   %2 corr von 1->2
Corr{1,2}.dst=frames{2}.pts;%(1:n*(2/3),:);
Corr{1,3}.src=frames{1}.pts;
Corr{1,3}.dst=frames{3}.pts;

Corr{2,1}.src=frames{2}.pts;%3 corr von 2->1
Corr{2,1}.dst=frames{1}.pts;
Corr{2,3}.src=frames{2}.pts;
Corr{2,3}.dst=frames{3}.pts;

Corr{3,1}.src=frames{3}.pts;
Corr{3,1}.dst=frames{1}.pts;
Corr{3,2}.src=frames{3}.pts;
Corr{3,2}.dst=frames{2}.pts;

frames{1}.fixed=true;
frames{2}.fixed=false;
frames{3}.fixed=false;
frames{1}.draw=@() 1+1;
frames{2}.draw=@() 1+1;
frames{3}.draw=@() 1+1;


mv_lm_icp_step(Corr,frames);
    
end