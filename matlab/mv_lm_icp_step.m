function [frames] = mv_lm_icp_step(Corr,frames)%poses,h,clouds,fixed)

n = length(frames);
lambda = 0.01;

for round=1:200
    
    %row number depends on #corr in each edge
    Jblock = sparse(0,6*n);%zeros(0,6*n);
    residualblock =zeros(0,1);
    
    JTest = zeros(0,n); %block structure of Jblock
    resTest = zeros(0,1);
    
    for i=1:n
        for j=1:n
            block = Corr{i,j};
            if(isempty(block) || frames{i}.fixed)
                continue;
            end
              
            src = project(frames{i}.pose,block.src);
            dst = project(frames{j}.pose,block.dst);
            
            [Jac, residual] = deriveErrorAnalytic3NJacobian(src,dst);   % x_hat=src, x=dst
                        
            Jblock(end+1:end+size(Jac,1),i*6-5:i*6)=Jac;
            residualblock(end+1:end+size(residual,1),:)=residual;
            
            JTest(end+1:end+2,i)=size(Jac,1);
            resTest(end+1:end+2,:)=size(residual,1);
            
        end
    end
    
    err = sum(residualblock.*residualblock) / n;
    
    disp(['error: ' num2str(err)]);
    
    H = Jblock'*Jblock;
    upd = - (H + lambda*eye(6*n) )^-1 * (Jblock') * residualblock;
    
    %TODO: only accept update if residual is reduced, increase/decrease
    %lambda
        
    for k=1:n
       xi=upd(k*6-5:k*6);
       frames{k}.pose = se3Exp(xi) * frames{k}.pose;
       frames{k}.draw();
       delete(frames{k}.h);
       frames{k}.h=plotCloud(project(frames{k}.pose,frames{k}.pts),'m');
    end
    
    
end

end
