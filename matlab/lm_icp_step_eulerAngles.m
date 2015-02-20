function [P] = lm_icp_step_eulerAngles(q,p)

p=p';
q=q';

Rx = @(a)[1     0       0;
          0     cos(a)  -sin(a);
          0     sin(a)  cos(a)];
      
Ry = @(b)[cos(b)    0   sin(b);
          0         1   0;
          -sin(b)   0   cos(b)];
      
Rz = @(g)[cos(g)    -sin(g) 0;
          sin(g)    cos(g)  0;
          0         0       1];

Rot = @(x)Rx(x(1))*Ry(x(2))*Rz(x(3));

myfun = @(x,xdata)Rot(x(1:3))*xdata+repmat(x(4:6),1,length(xdata));


options = optimset('Algorithm', 'levenberg-marquardt');
x = lsqcurvefit(myfun, zeros(6,1), p, q, [], [], options);


R = Rot(x(1:3));
T = x(4:6);

P = ones(4);
P(1:3,1:3) = R;
P(1:3,4) = T;

