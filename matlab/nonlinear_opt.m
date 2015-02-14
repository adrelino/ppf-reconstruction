function [] = nonlinear_opt()
close all;
addpath('../PointPairFeaturesImpl');
dir = '/Users/adrian/git/point-pair-features/samples/Bunny_RealData/';

for k=1:2
    
    numb = (k-1)*3;
    
    ks=num2str(numb,'%0.6d');
    
    file=[dir 'cloudXYZ_' num2str(numb) '.xyz'];

    clouds{k} = dlmread(file);
    
    posefile = [dir 'estimates_' ks '.txt'];

    poses{k} = dlmread(posefile);
end

% poses{k}(1,4) = poses{k}(1,4);
% 
% cloud1 = clouds{k}(:,1:3);%project(poses{k},clouds{k});
T = eye(4);
T(1:3,4)=[0.01,0.02,0.03];
% %P=poses{k}*T;
% cloud2 = project(T,clouds{k});

cloud1 = project(poses{k}*T,clouds{k});
cloud2Orig = project(poses{k-1},clouds{k-1});
cloud2=cloud2Orig;


close all;
figure;
axis vis3d;
plotCloud(cloud1,'b');
hold on;
plotCloud(cloud2Orig,'r');
hold on;

h=0;

for round=1:50
    tic
    [idx,dist] = knnsearch(cloud1,cloud2); %size(idx) = rows(cloud2)
    toc
        
    thresh = 0.1; % 5mm
    foo = dist <= thresh;

    pts = cloud2(foo,:);
    ptsRef = cloud1(idx(foo),:);
    
    numCorr = length(pts)


    %Test = lm_icp_step_eulerAngles(ptsRef,pts);
    Test = lm_icp_step_twistsSimple(ptsRef,pts);
    cloud2=project(Test,cloud2);
    
    if(ishandle(h) && round > 1)
        delete(h);
    end
    h=plotCloud(cloud2,'g');
    drawnow;

    diffVec=ptsRef - cloud2(foo,:);
    err = sqrt(sum(diffVec .* diffVec,2));

    summedErr = sum(err)

end
    
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