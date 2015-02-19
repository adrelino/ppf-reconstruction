function [P] = mv_lm_icp_step(Corr,poses,h,clouds,fixed)

n = length(poses);

%xi = zeros(6*n);



lambda = 0.01;

for round=1:50
  Hblock = zeros(6*n);  %block structure is graph adjacency matrix
  dblock = zeros(6*n,1);
    for i=1:n
        for j=1:n
            block = Corr{i,j};
            if(isempty(block))
                continue;
            end
              
            src = project(poses{i},block.src);
            dst = project(poses{j},block.dst);
            
            [Jac, residual] = deriveErrorAnalytic(src,dst);   % ENABLE ME FOR ANALYTIC DERIVATIVES
            
            d = Jac' * residual; %gradient
            dblock(i*6-5:i*6) = dblock(i*6-5:i*6)+d;
            
            H = Jac' * Jac; % Hessian approximation
            R = eye(size(H,1)); %Levenberg original
            H = H + lambda * R; %regu

            
            Hblock(i*6-5:i*6,j*6-5:j*6)=H;
        end
    end
    
    %Hblock = Hblock + lambda * eye(size(Hblock));
    
    upd = - Hblock^-1 * dblock;
    
    for k=1:n
       xi=upd(k*6-5:k*6);
       poses{k} = se3Exp(xi) * poses{k};
       delete(h(k));
       h(k)=plotCloud(project(poses{k},clouds{k}),'m');
    end
    
    
end



end

function [ D ] = project(P,C)
    Cok=[C(:,1:3) ones(size(C,1),1)]';
    Dbad=(P*Cok)';
    D=Dbad(:,1:3); %D=D(1:end,:);
end


function [ err ] = calcError(ptsRef, ptsProj)
    diffVec=ptsProj - ptsRef;
    err = dot(diffVec,diffVec,2);
end

function [ Jac, residual ] = deriveErrorAnalytic(ptsRef, ptsProj)
    Jac = zeros(size(ptsRef,1),6);
    
    diffVec=ptsProj - ptsRef;
    residual = dot(diffVec,diffVec,2);
    
    for i=1:size(ptsRef,1)
            p2=ptsProj(i,:);
            d = [eye(3) -hat(p2)];
            
            pdiff=diffVec(i,:);
            
            bla=pdiff*d;
            
            Jac(i,:)=2*bla;
    end
    
end