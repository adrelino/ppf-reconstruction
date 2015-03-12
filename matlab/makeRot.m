y = [0,1,0]';
normal = [0.5 0.2 0.3]';

R = zeros(3);
R(3,:) = normal;
y = y - normal(1)*normal;
y = y / norm(y);
R(2,:) = y';
R(1,:) = cross(normal,y);


R*normal