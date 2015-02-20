function [ Jacc, residuall] = deriveErrorAnalytic3NJacobian(x_hat, x)    
    n=size(x,1);
    epsVec=x_hat - x;
    residuall = reshape(epsVec',[3*n,1]);
        
    xdata=x_hat';   %or is it x--> converges faster with just x
    Jacc = zeros(3*n,6);
    Jacc(1:3:(3*n),:)=[repmat([1 0 0],n,1) zeros(n,1) xdata(3,:)' -xdata(2,:)'];
    Jacc(2:3:(3*n),:)=[repmat([0 1 0],n,1) -xdata(3,:)' zeros(n,1) xdata(1,:)'];
    Jacc(3:3:(3*n),:)=[repmat([0 0 1],n,1) xdata(2,:)' -xdata(1,:)' zeros(n,1)];
    
end