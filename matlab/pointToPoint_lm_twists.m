function T = pointToPoint_lm_twists(src, dst)
    N = size(src,1); assert(size(dst,1) == N);
    T = eye(4);  x_hat=src;   lambda=1; 

for i = 1:100
    %stack distance vectors into residual vector
    e = reshape((x_hat - dst)',[3*N,1]);
    
    %stack Jacobian
    J = zeros(3*N,6); O = zeros(N,1); I = ones(N,1);
    x1 = x_hat(:,1); x2 = x_hat(:,2); x3 = x_hat(:,3);
    J(1:3:3*N,:) = [I O O   O  x3 -x2];
    J(2:3:3*N,:) = [O I O -x3   O  x1];
    J(3:3:3*N,:) = [O O I  x2  -x1  O];
    
    b = (J' * e); %gradient
    
    if(max(abs(b))< 0.00001)
        break; %convergence in gradient, lambda is big
    end
    
    %Levenberg step
    upd = -(J' * J + lambda*eye(6))^(-1) * b; 

    %compute new residual vector
    T_new = se3Exp(upd) * T; %exponential map left multiply
    R = T_new(1:3,1:3); t = T_new(1:3,4);
    x_hat_new = (R*src' + repmat(t,1,N))'; %project
    e_test = reshape((x_hat_new - dst)',[3*N,1]);
    
    %test if we reduced the error (sum of squared residuals)
    if(e_test'*e_test < e'*e) %error reduced
        T = T_new; x_hat = x_hat_new; %accept update
        lambda = lambda / 10; %towards Gauss-Newton
    else %error increased, discard update
        lambda = lambda * 10; %towards gradient descent
    end
end