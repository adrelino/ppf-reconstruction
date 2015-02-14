function [P] = lm_icp_step_twistsSimple(q,p)

p=p';
q=q';

myfun = @(x,xdata) project2(se3Exp(x),xdata);


options = optimset('Algorithm', 'levenberg-marquardt');
x = lsqcurvefit(myfun, zeros(6,1), p, q, [], [], options);

P = se3Exp(x);

end


function [ D ] = project2(P,C)
    Cok=[C(1:3,:); ones(1,size(C,2))];
    Dbad=(P*Cok);
    D=Dbad(1:3,:);
end
