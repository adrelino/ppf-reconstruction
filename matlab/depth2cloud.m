function [cloud, ordered]= depth2cloud(depth, fx, fy,cx,cy)
[m,n] = size(depth);

X = (ones(m,1)*(1:n) - cx).*depth*(1/fx);
Y = ((1:m)'*ones(1,n) - cy).*depth*(1/fy);
ordered = cat(3,X,Y,depth);

cloud = [reshape(X,m*n,1) reshape(Y,m*n,1) reshape(depth,m*n,1)];
cloud( ~any(cloud,2), : ) = []; %remove zero points

end

