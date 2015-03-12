function [ T ] = se3Exp( xi )
    xi_hat = [0 -xi(6) xi(5) xi(1);
              xi(6) 0 -xi(4) xi(2);
              -xi(5) xi(4) 0 xi(3);
              0      0     0    0];
    T = expm(xi_hat);
end

