function [Xdot,v,w] = NL_control(t,X,x_star,y_star,theta_star,xd_star,yd_star,xdd_star,ydd_star,k1,k2,k3)
%stato del robot
x = X(1);
y = X(2);
theta = X(3);

v_star = sqrt(xd_star(t).^2+yd_star(t).^2);
%w_star = thetad_star
w_star = (ydd_star(t).*xd_star(t)-yd_star(t).*xdd_star(t))./(v_star.^2);

%errore
ex = x_star(t)-x;
ey = y_star(t)-y;
etheta = atan2(sin(theta_star(t)-theta),cos(theta_star(t)-theta));

%applicazione legge
u1 = -k1(v_star,w_star).*ex;
u2 = -k2*v_star.*(sin(etheta)./etheta).*ey - k3(v_star,w_star).*etheta;
v = v_star.*cos(theta)-u1;
w = w_star - u2;

%evoluzione modello DD 
Xdot = [v*cos(theta);
        v*sin(theta);
        w];


end