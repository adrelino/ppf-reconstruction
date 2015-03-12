function [ T ] = pointToPoint(src,dst)
    N = size(src,1); assert(size(dst,1) == N);
    ps = src';
    qs = dst'; %ps and qs have 3 rows, N columns
    p_dash = mean(ps,2);
    q_dash = mean(qs,2);
    ps_centered = ps - repmat(p_dash,1,size(ps,2));
    qs_centered = qs - repmat(q_dash,1,size(qs,2));
    K = qs_centered * ps_centered';
    [U,~,V] = svd(K);
    R = U * V';
    if(det(R)<0)
        R(:,3) = R(:,3) * (-1);
    end
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = q_dash - R*p_dash;
end