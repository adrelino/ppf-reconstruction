function [T] = eulerAngle(z,y,x)

Z = eye(3);
Z(2:3,2:3)=euler2x2(z);
Y = eye(3);
Y([1 3],[1 3])=-euler2x2(y);
X = eye(3);
X(1:2,1:2)=euler2x2(x);

T = Z*Y*X;

end


function [ m ] = euler2x2( alpha )
m = [cos(alpha), sin(alpha);
     -sin(alpha), cos(alpha)];
end

