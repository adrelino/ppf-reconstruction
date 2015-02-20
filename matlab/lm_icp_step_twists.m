function T = lm_icp_step_twists(src, dst) %   T : src -> dst

T = eye(4);
%fitzgibbon, 2.2
% The algorithm is not very sensitive to the choice of ?, but as a rule of thumb, one should
% use a small value, eg ? = 10?6 if
% x0 is believed to be a good approximation
% Otherwise, use ? = 10?3 or even ? = 1.
lambda=1;
tau=1;%10e-3;


x_hat=src;
x=dst;

a=plotCloud(x,'g');hold on;
b=plotCloud(x_hat,'b');hold on;

h=plotCloud(x_hat,'r');hold on;
% eps = x_hat - x;   x_hat estimated, x measured


eps1=0.00001; %convergence in gradient
eps3=0.00001; %convergence in impr %
eps4=0.00001; %convergence in last error dif to initial error


for i=1:200
    
    % calculate Jacobian of residual function (Matrix of dim (width*height) x 6)
   
    %[Jac, residual] = deriveErrorNumeric(ptsRef,pts);   % ENABLE ME FOR NUMERIC DERIVATIVES
    %gradient step seams good
    %[Jac, residual, Jacc, residuall] = deriveErrorAnalytic(x_hat,x);   % ENABLE ME FOR ANALYTIC DERIVATIVES
    %converges faster
    [Jac, residual] = deriveErrorAnalytic3NJacobian(x_hat,x);
    
    
    %residual contains the squared diffVec norms
    %sum(residual)==residuall'*residuall
    errLast = mean(calcError(x_hat,x));   %mean(residual);
    if(i==1)
        errInitial=errLast;
    end

    
    % do Gauss-Newton step
    
   R = eye(6); %Levenberg original
%     R = diag(diag(H)); %Levenberg-Marquardt --> uses curvature information to have large gradient even in small valleys
%     if(i==1)
%        lambda = tau * max(diag(H)); 
%     end

    d=Jac' * residual; %gradient
    H = Jac' * Jac;
    upd = -(( H + lambda * R)^-1) * d;

    
    %see if we reduced the mean error
    XiTest = se3Exp(upd) * T;
    x_hat_test=project(XiTest,src);
    
    delete(h);
    h=plotCloud(x_hat_test,'r');hold on;
    drawnow;
    %pause(0.5);
    
    errTest = mean(calcError(x_hat_test,x));    
    
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
        
        T = XiTest;
        x_hat=x_hat_test;
        %errLast = errTest;


        %pause(0.00005);
        
        if(impr < eps3)
            disp('convergence in impr');
            break;
        end
          
    end
    
end
disp('finish');
delete(a);
delete(b);
delete(h);
end
    
% Determine the RMS error between two point equally sized point clouds with
% point correspondance.
% ER = rms_error(p1,p2) where p1 and p2 are 3xn matrices.
function ER = rms_error(p1,p2)
    dsq = sum(power(p1 - p2, 2),1);
    ER = sqrt(mean(dsq));
end

function [ err ] = calcError(x_hat, x)
    diffVec=x_hat - x;
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

function [ Jac, residual,Jacc,residuall ] = deriveErrorAnalytic(x_hat, x)
    Jac = zeros(size(x,1),6);
    
    epsVec=x_hat - x;
    residual = dot(epsVec,epsVec,2);
    Jacc = zeros(0,6);
    residuall = reshape(epsVec',[3*size(x,1),1]);
    
    for i=1:size(x,1)
            p2=x_hat(i,:);
            d = [eye(3) -hat(p2)];
            
            Jacc(end+1:end+3,:)=d;
            
            pdiff=epsVec(i,:);
            
            bla=pdiff*d;
            
            Jac(i,:)=2*bla;
    end
    
end
