function [h] = plotCloud(cloud,color)
   h=plot3(cloud(:,1),cloud(:,2),cloud(:,3),'.','Color',color);
   hold on;
end