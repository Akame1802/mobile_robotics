function [Xdot,v,w] = lin_control(t,X,x_star,y_star,theta_star,xd_star,yd_star,xdd_star,ydd_star,a,delta)
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
k1 = 2*delta*a; k3=k1;
k2 = (a.^2-w_star.^2)./v_star;
v = v_star.*cos(etheta)-k1*ex;
w = w_star + k2.*ey + k3*etheta;

%evoluzione modello DD 
Xdot = [v*cos(theta);
         v*sin(theta);
         w];
end