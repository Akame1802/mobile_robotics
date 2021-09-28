function [Xdot,v,w] = FL_control(t,X,x_star,y_star,xd_star,yd_star,k1,k2,b)
%stato del robot
x = X(1);
y = X(2);
theta = X(3);

xB = x+b*cos(theta);
yB = y+b*sin(theta);
% [xdotB;ydotB] = T(theta)*[v;w]
% [v;w] = inv(T(theta))*[u1;u2]

Tinv = [cos(theta), sin(theta);
        -sin(theta)/b, cos(theta)/b];

u1 = xd_star(t)+k1*(x_star(t)-xB); %u1=xdotB
u2 = yd_star(t)+k2*(y_star(t)-yB); %u2=ydotB
vect = Tinv*[u1;u2];

v = vect(1,:);
w = vect(2,:);

Xdot = [v*cos(theta);
        v*sin(theta);
        w];

end