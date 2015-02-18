function [P] = lm_icp_step_twistsSimple(q,p)

p=p';
q=q';

myfun = @(x,xdata) project2(se3Exp(x),xdata);

%options = optimoptions('lsqcurvefit')
% tic
% options = optimset('Algorithm', 'levenberg-marquardt');
% x1 = lsqcurvefit(myfun, zeros(6,1), p, q, [], [], options);
% toc

tic
options = optimset('Algorithm', 'levenberg-marquardt','Jacobian','on');
x = lsqcurvefit(@myfun2, zeros(6,1), p, q, [], [], options);
toc

%norm(x1-x)

P = se3Exp(x);

end

function [F,Jacc] = myfun2(x,xdata)
%disp(x)
F = project2(se3Exp(x),xdata);          % objective function values at x

if nargout > 1   % two output arguments
    % Jacobian of the function evaluated at x
   
    n=size(xdata,2);
%     Jacc = zeros(n*3,6);
%     
%     for i=1:n
%             p2=xdata(:,i);
%             d = [eye(3) -hat(p2)];
%             Jacc(3*i-2:(3*i),:)=d;
%     end
    
    %disp(size(Jac));
    Jacc = zeros(n*3,6);
    Jacc(1:3:(3*n),:)=[repmat([1 0 0],n,1) zeros(n,1) xdata(3,:)' -xdata(2,:)'];
    Jacc(2:3:(3*n),:)=[repmat([0 1 0],n,1) -xdata(3,:)' zeros(n,1) xdata(1,:)'];
    Jacc(3:3:(3*n),:)=[repmat([0 0 1],n,1) xdata(2,:)' -xdata(1,:)' zeros(n,1)];
   
end

end


function [ D ] = project2(P,C)
    Cok=[C(1:3,:); ones(1,size(C,2))];
    Dbad=(P*Cok);
    D=Dbad(1:3,:);
end
