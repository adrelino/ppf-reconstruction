function [] = lm_icp_step_twists(pts, ptsRef)

% initialization
xi = [0 0 0 0 0 0]'; %twist

% just do at most 20 steps.
errLast = 1e10;
for i=1:10

    % calculate Jacobian of residual function (Matrix of dim (width*height) x 6)
    [Jac, residual] = deriveErrorNumeric(ptsRef,pts,xi);   % ENABLE ME FOR NUMERIC DERIVATIVES
   %[Jac, residual] = deriveErrAnalytic(IRef,DRef,I,xi,Klvl);   % ENABLE ME FOR ANALYTIC DERIVATIVES

    % just take the pixels that have no NaN (e.g. because
    % out-of-bounds, or because the didnt have valid depth).
    valid = ~isnan(sum(Jac,2));
    residualTrim = residual(valid,:);
    JacTrim = Jac(valid,:);

    % do Gauss-Newton step
    upd = - (JacTrim' * JacTrim)^-1 * JacTrim' * residualTrim;

    % MULTIPLY increment from left onto the current estimate.
    xi = se3Log(se3Exp(upd) * se3Exp(xi))

    % get mean and display
    err = mean(residualTrim .* residualTrim)


    Test = se3Exp(xi);
    plotCloud(project(Test,pts),'r');
    hold on;
    plotCloud(ptsRef,'b');
    axis vis3d;
    hold off;


    %calcErr(c1,d1,c2,xi,K);
    pause(0.1);

    % break if no improvement
%         if(err / errLast > 0.99)
%             break;
%         end
    errLast = err;
end
end
    
    
    
% Determine the RMS error between two point equally sized point clouds with
% point correspondance.
% ER = rms_error(p1,p2) where p1 and p2 are 3xn matrices.
function ER = rms_error(p1,p2)
    dsq = sum(power(p1 - p2, 2),1);
    ER = sqrt(mean(dsq));
end

function [ err ] = calcError(ptsRef, pts, xi)
    T = se3Exp(xi);
    ptsProj = project(T,pts);
    diffVec=ptsRef - ptsProj;
    err = sqrt(sum(diffVec .* diffVec,2));
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
        diff = (residual - residualPerm);
        Jac(:,j) = diff / eps;
    end
end