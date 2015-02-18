function [P,h] = lm_icp_step_twists(ptsRef, pts,h)

% initialization
xi = [0 0 0 0 0 0]'; %twist

% just do at most 20 steps.
errLast = 200000;
%tau=0.1;

%fitzgibbon, 2.2
lambda=0.1;

lambdaFac=2;

for i=1:50
    
    Test = se3Exp(xi);
    
    
    pts2=project(Test,pts);
    
    delete(h);
    h=plotCloud(pts2,'g');
    hold on;

    pause(0.1);

    % calculate Jacobian of residual function (Matrix of dim (width*height) x 6)
    [Jac, residual] = deriveErrorNumeric(ptsRef,pts,xi);   % ENABLE ME FOR NUMERIC DERIVATIVES
    %[Jac, residual] = deriveErrorAnalytic(ptsRef,pts,xi);   % ENABLE ME FOR ANALYTIC DERIVATIVES

   

    
    % do Gauss-Newton step
    JTJ = Jac' * Jac;
    %Quad = ( JTJ)^-1;
    Quad = ( JTJ + lambda * eye(6))^-1;
    Quad = real(Quad);
    upd = - Quad * Jac' * residual;
    
    %upd = - Jac' * residual;

    % MULTIPLY increment from left onto the current estimate.
    xi = real(se3Log(se3Exp(upd) * se3Exp(xi)));



    % get mean and display
    err = mean(residual .* residual); %mean
    disp(['i: ' num2str(i) ' err: ' num2str(err) ' lambda: ' num2str(lambda)]);

       % break if no improvement
%      if(abs(err-errLast) < 1e-02)%err / errLast > 0.99)
%          disp('Optmization stopped err-errLast < 1e-6');
%          break;
%      end
%     elseif(err / errLast > 0.6)
%        lambda = lambda * lambdaFac; 
%     elseif (err / errLast < 0.4)
%         lambda = lambda / lambdaFac;
%    
    %lambda = i*50;

    
    errLast = err;
end
P = se3Exp(xi);
end
    
% Determine the RMS error between two point equally sized point clouds with
% point correspondance.
% ER = rms_error(p1,p2) where p1 and p2 are 3xn matrices.
function ER = rms_error(p1,p2)
    dsq = sum(power(p1 - p2, 2),1);
    ER = sqrt(mean(dsq));
end

function [ D ] = project(P,C)
    Cok=[C(:,1:3) ones(size(C,1),1)]';
    Dbad=(P*Cok)';
    D=Dbad(:,1:3); %D=D(1:end,:);
end



function [ err ] = calcError(ptsRef, pts, xi)
    T = se3Exp(xi);
    ptsProj = project(T,pts);
    diffVec=ptsProj - ptsRef;
    err = dot(diffVec,diffVec,2);
end

function [ Jac, residual ] = deriveErrorNumeric(ptsRef, pts, xi)
    eps = 1e-8;
    Jac = zeros(size(pts,1),6);
    residual = calcError(ptsRef,pts,xi);
    for j=1:6
        epsVec = zeros(6,1);
        epsVec(j) = eps;
        
        % MULTIPLY epsilon from left onto the current estimate.
        xiPerm =  se3Log(se3Exp(epsVec) * se3Exp(xi));
        xiPerm = real(xiPerm);
        residualPerm = calcError(ptsRef,pts,xiPerm);
        diff = (residualPerm - residual);
        Jac(:,j) = diff / eps;
    end
end

function [ Jac, residual ] = deriveErrorAnalytic(ptsRef, pts, xi)
    Jac = zeros(size(pts,1),6);
    T = se3Exp(xi);
    ptsProj = project(T,pts);
    
    diffVec=ptsProj - ptsRef;
    residual = dot(diffVec,diffVec,2);
    
    for i=1:size(pts,1)
            p2=ptsProj(i,:);
            d = [eye(3) -hat(p2)];
            
            pdiff=diffVec(i,:);
            
            bla=pdiff*d;
            
            Jac(i,:)=2*bla;

        
        
    end
    
    
    %Jac(:,1) = 
    
end