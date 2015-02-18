function [] = nonlinear_opt_easy()
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

ptsPreProj = ptsRef;
noise = randn(n,3)*0.01;
noise(:,3) = noise(:,3)*zNoise;
noise(:,2) = noise(:,2)*yNoise;
ptsPreProj = ptsPreProj+noise;

pts = project(T,ptsPreProj);


    
plotCloud(ptsRef,'b');hold on;
plotCloud(pts,'r');hold on;
h=plotCloud(pts,'g');hold on;


Ref = lm_icp_step_twistsSimple(pts,ptsRef);
plotCloud(project(Ref,ptsRef),'m');
%Test = lm_icp_step_twists(pts,ptsRef,h);

disp(T);
disp(Ref);
%disp(Test);
    
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