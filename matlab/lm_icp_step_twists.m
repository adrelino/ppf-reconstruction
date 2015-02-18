function [Xi,h] = lm_icp_step_twists(ptsRef, ptsOrig,h)

% initialization
%xi = [0 0 0 0 0 0]'; %twist

Xi = eye(4);

%fitzgibbon, 2.2
lambda=100;

pts=ptsOrig;

%lambdaFac=2;
% 
% The algorithm is not very sensitive to the choice of ?, but as a rule of thumb, one should
% use a small value, eg ? = 10?6 if
% x0 is believed to be a good approximation to
% x?.
% Otherwise, use ? = 10?3 or even ? = 1.

tau=10e-3;

eps1=0.000001; %convergence in gradient
eps3=0.00001; %convergence in impr %
eps4=0.00001; %convergence in last error dif to initial error


for i=1:200
    
    % calculate Jacobian of residual function (Matrix of dim (width*height) x 6)
    %[Jac, residual] = deriveErrorNumeric(ptsRef,pts);   % ENABLE ME FOR NUMERIC DERIVATIVES
    
    [Jac, residual] = deriveErrorAnalytic(ptsRef,pts);   % ENABLE ME FOR ANALYTIC DERIVATIVES
    
    %JT=Jac/Jac2;
    
    
    %residual contains the squared diffVec norms
    
    errLast = mean(residual);
    if(i==1)
        errInitial=errLast;
    end

    d = Jac' * residual; %gradient

    
    % do Gauss-Newton step
    
    H = Jac' * Jac; % Hessian approximation
    R = eye(size(H,1)); %Levenberg original

    R = diag(diag(H)); %Levenberg-Marquardt --> uses curvature information to have large gradient even in small valleys
    if(i==1)
       lambda = tau * max(diag(H)); 
    end
    Quad = ( H + lambda * R)^-1;
    upd = - Quad * d;
    
    % gradient descent
    %upd = -d;

    % MULTIPLY increment from left onto the current estimate.
    %xi = real(se3Log(se3Exp(upd) * se3Exp(xi)));
    
    %see if we reduced the mean error
    XiTest = se3Exp(upd) * Xi;
    ptsTest=project(XiTest,ptsOrig);
    
    residualTest = calcError(ptsRef,ptsTest);
    errTest = mean(residualTest);
    
    
    impr = (1 - errTest / errLast) * 100;
    
    if( max(abs(d)) < eps1)
        disp('convergence in gradient');
        break;
    end
    
    if( abs(errLast - errTest) / errInitial <eps4)
           disp('convergence in final change in the mean of squares relative to initial value');
           break;
    end
    

    
    if(impr < 0)  %no improvement, went to local maximum
        disp(['-i: ' num2str(i) ' err: ' num2str(errTest) ' impr: ' num2str(impr) ' lambda: ' num2str(lambda)]);
        lambda = lambda * 10;
    else
        disp(['+i: ' num2str(i) ' err: ' num2str(errTest) ' impr: ' num2str(impr) ' lambda: ' num2str(lambda)]);
        lambda = lambda / 10;
        
        Xi = XiTest;
        pts=ptsTest;
        %errLast = errTest;

        delete(h);
        h=plotCloud(pts,'g');
        hold on;
        drawnow;
        %pause(0.00005);
        
        if(impr < eps3)
            disp('convergence in impr');
            break;
        end
          
    end
    
    end
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



function [ err ] = calcError(ptsRef, ptsProj)
    diffVec=ptsProj - ptsRef;
    err = dot(diffVec,diffVec,2);
end

function [ Jac, residual ] = deriveErrorNumeric(ptsRef, pts)
    eps = 1e-8;
    Jac = zeros(size(pts,1),6);
    residual = calcError(ptsRef,pts);
    for j=1:6
        epsVec = zeros(6,1);
        epsVec(j) = eps;
        
        % MULTIPLY epsilon from left onto the current estimate.
        TPerm =  se3Exp(epsVec);
        ptsPerm = project(TPerm,pts);

        residualPerm = calcError(ptsRef,ptsPerm);
        
        diff = (residualPerm - residual);
        Jac(:,j) = diff / eps;
    end
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