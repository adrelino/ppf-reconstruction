classdef Frame < handle
    %one frame
    
    properties
        pts;
        pose;
        poseG;
        fixed;
        h;
        hh;
        k;
        neighbours;
    end
    
    methods
        function obj=Frame(pts,pose,k)
            obj.pts=pts;
            obj.pose=pose;
            obj.fixed=false;
            obj.k=k;
            obj.hh=scatter3(0,0,0); hold on;
        end
        
        function draw(obj)
            delete(obj.hh);
            cams=obj.pose(1:3,4)';
            obj.hh(1)=text(cams(1),cams(2),cams(3),num2str(obj.k),'Color','r');hold on;
            camsG=obj.poseG(1:3,4)';
            obj.hh(2)=text(camsG(1),camsG(2),camsG(3),num2str(obj.k),'Color','g');hold on;
        end
    end
    
end

