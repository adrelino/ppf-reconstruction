clear all
syms px py pz qx qy qz nx ny nz real
p = [px py pz]';
q = [qx qy qz]';
n = [nx ny nz]';

%https://www.iscs.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
%s=p, d=q
a=cross(p,n);
A = [ a' n' ]; %(10)
b = -((p-q)'*n)';


%http://www.cs.princeton.edu/~smr/papers/icpstability.pdf
c = cross(p,n);
f = (p-q)'*n;

d = -[
    c(1)*f;
    c(2)*f;
    c(3)*f;
    n(1)*f;
    n(2)*f;
    n(3)*f;
    ];

C = [c' n']'*[c' n']';

% A'Ax=A'b <==> Cx=d are they equal?
isequaln(d,A'*b)
isequaln(C,A'*A)
