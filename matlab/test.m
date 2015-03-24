n = 100000; %1e6;
m = 3;
iter = 900;
a = rand(1,m);
b = rand(n,m);

c = zeros(size(b));
tic
for i = 1:iter
    c(:,1) = b(:,1) - a(1);
    c(:,2) = b(:,2) - a(2);
    c(:,3) = b(:,3) - a(3);
end
toc

on=ones(n,1);
tic
for i = 1:iter
    c = b-a(on,:);
end
toc

tic
for i = 1:iter
    c = b-repmat(a,size(b,1),1);
end
toc

tic
for i = 1:iter
    c = bsxfun(@minus,b,a);
end
toc