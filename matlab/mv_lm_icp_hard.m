function [] = mv_lm_icp_hard()

dir = '/Users/adrian/git/point-pair-features/samples/Bunny_RealData/';
close all;
figure;
axis vis3d;
xlim([-0.5,0.5])
ylim([0,0.35])
zlim([-0.5,0.5])

n=37;
colors = jet(n);

cams = zeros(n,3);

for k=1:n
    
    numb = (k-1);
    
    ks=num2str(numb,'%0.6d');
    
    file=[dir 'cloudXYZ_' num2str(numb) '.xyz'];

    cloud = dlmread(file);
    
    posefile = [dir 'estimates_' ks '.txt'];

    pose = dlmread(posefile);
    
    posefile2 = [dir 'poses_' num2str(numb) '.txt'];
    poseG = dlmread(posefile2);
    
    cams(k,:)=pose(1:3,4)';
        
    frames{k}=Frame(cloud(:,1:3),pose,k);
    frames{k}.poseG=poseG;
    
    
    frames{k}.draw();

end

frames{1}.fixed=true;

%pose neighbour search
knn=5;
[idxCams,distCams] = knnsearch(cams,cams,'K',knn+1); %first column is cam itself
neighbours = idxCams(:,2:end);

Corr=cell(n,n); %adjacency matrix

for k=1:n
    frames{k}.neighbours=neighbours(k,:);
    frame_x_hat=frames{k};  %cloud2 is moving to all neighbours
    
    if(frame_x_hat.fixed)
        continue;
    end

    for j=1:length(frame_x_hat.neighbours);
        
        idxN=frame_x_hat.neighbours(j);

        frame_x=frames{idxN};  %all neighbours stay fixed for now

        [idx,dist] = knnsearch(frame_x.pts,frame_x_hat.pts); %size(idx) = rows(cloud2)

        thresh = 0.005; % 5mm
        foo = dist <= thresh;

        Corr{k,idxN}.src = frame_x_hat.pts(foo,:);
        Corr{k,idxN}.dst = frame_x.pts(idx(foo),:);

        %numCorr = length(pts)
    end %end neighbours
    
end %frames

mv_lm_icp_step(Corr,frames);

end