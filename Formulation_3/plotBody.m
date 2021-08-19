function plotBody(P, theta)
%PLOTBODY Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1
    theta = 0;
end
R = RotationMatrix(theta);

V = cumsum(R*[[0;0],-P.r_l_2, -P.r_l_1, [P.t; 0], P.r_a_1, P.r_a_2],2);
plot(V(1,:),V(2,:), '-b');
hold on

V = cumsum(R*[[0;0], -P.r_COM_f, P.r_COM_h],2);
plot(V(1,:),V(2,:), '-g');
plot(V(1,2),V(2,2),'.r','MarkerSize',20)

V = cumsum(R*[[0;0], P.r_f_h],2);
plot(V(1,:),V(2,:), '-k');

hold off
axis equal
end

