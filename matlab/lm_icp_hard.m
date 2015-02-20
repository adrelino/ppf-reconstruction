function [] = lm_icp_hard()
%close all;
%addpath('../PointPairFeaturesImpl');
dir = '/Users/adrian/git/point-pair-features/samples/Bunny_RealData/';


%close all;
figure;
axis vis3d;

n=2;
colors = jet(n);

cams = zeros(n,3);
camsG = zeros(n,3);


for k=1:n
    
    numb = (k-1);
    
    ks=num2str(numb,'%0.6d');
    
    file=[dir 'cloudXYZ_' num2str(numb) '.xyz'];

    clouds{k} = dlmread(file);
    
    posefile = [dir 'estimates_' ks '.txt'];

    poses{k} = dlmread(posefile);
    
    posefile2 = [dir 'poses_' num2str(numb) '.txt'];
    posesGroundTruth{k} = dlmread(posefile2);

    
    if(k==n)
        T = eye(4);
        T(1:3,4)=[0.01,0.02,0.03];
        %T(1:3,4)=[0.1,0.5,3];
        %T(1:3,1:3)=eulerAngle(pi/19,-pi*0.001,pi/23);
        poses{k}=poses{k}*T;
    end
    
    cams(k,1:3)=poses{k}(1:3,4)';
    camsG(k,1:3)=posesGroundTruth{k}(1:3,4)';

    
    cloudsPr{k}=project(poses{k},clouds{k});
    

    
    h{k}=plotCloud(cloudsPr{k},colors(k,:));
    hold on;

end

% po=plot3(cams(:,1),cams(:,2),cams(:,3));
% hold on;
% plot3(camsG(:,1),camsG(:,2),camsG(:,3),'g');
% hold on;

%poses{k}(1,4) = poses{k}(1,4);
%cloud1 = clouds{k}(:,1:3);%project(poses{k},clouds{k});
% T = eye(4);
% T(1:3,4)=[0.01,0.02,0.03];
%P=poses{k}*T;

% cloud1 = project(poses{k},clouds{k});
% cloud2Orig = project(poses{k-1},clouds{k-1});
% cloud2= project(T,cloud2Orig);

for round=1:50
    
    for k=2:n
        
    cloud1=cloudsPr{k-1};
    cloud2=cloudsPr{k};

    [idx,dist] = knnsearch(cloud1,cloud2); %size(idx) = rows(cloud2)
        
    thresh = 0.1; % 5mm
    foo = dist <= thresh;

    pts = cloud2(foo,:);
    ptsRef = cloud1(idx(foo),:);
    
    %numCorr = length(pts)


    %Test = lm_icp_step_eulerAngles(ptsRef,pts);
    %Test = lm_icp_step_twistsSimple(ptsRef,pts);

    Test = lm_icp_step_twists(pts,ptsRef);%    h{k}=h2;
    cloud2=project(Test,cloud2);
    cloudsPr{k}=cloud2;
    
    poses{k}=Test * poses{k};
    
    cams(k,1:3)=poses{k}(1:3,4)';


    
    delete(h{k});
    h{k}=plotCloud(cloud2,colors(k,:));
    drawnow;

    diffVec=ptsRef - cloud2(foo,:);
    err = mean(sum(diffVec .* diffVec,2));
    
    disp(['round: ' num2str(round) ' error: ' num2str(err)]);

    end
    
%     delete(po);
%     po=plot3(cams(:,1),cams(:,2),cams(:,3),'r');
%     hold on;
%     drawnow;
    
    disp(round);
    

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