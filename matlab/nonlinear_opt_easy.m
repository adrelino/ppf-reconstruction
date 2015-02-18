function [] = nonlinear_opt_easy()
close all;
axis vis3d;   

T = eye(4);
T(1:3,4)=[0.1,0.5,3];
T(1:3,1:3)=eulerAngle(pi/4,-pi*0.9,pi/2);

ptsRef = randn(100,3);
ptsRef(:,3) = ptsRef(:,3)*0.001;
pts = project(T,ptsRef);
    
plotCloud(ptsRef,'b');hold on;
plotCloud(pts,'r');hold on;
h=plotCloud(pts,'g');hold on;

    
%Test = lm_icp_step_eulerAngles(ptsRef,pts);
%Test = lm_icp_step_twists(ptsRef,pts);

[Test] = lm_icp_step_twists(ptsRef,pts,h);

disp(T);
disp(Test);
    
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