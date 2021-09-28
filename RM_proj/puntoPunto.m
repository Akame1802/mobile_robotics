function X = puntoPunto(traj_points)
tc = 0.1; %tempo di campionamento
%npoints/Tmax = tc
Tmax = min(50,(size(traj_points,1)-1)/tc); %Tmax senza considerare i tempi di rotazione sul posto

%calcolo dei tempi di percorrenza di ciascun tratto
lengths = zeros(1, size(traj_points,1)-1); %lunghezze dei tratti percorsi
for k=1:length(lengths)
    p1 = traj_points(k,:);
    p2 = traj_points(k+1,:);
    lengths(k) = pdist([p1;p2],'Euclidean');
end
%times(k) è il tempo necessario a percorrere il tratto k con Tmax fissato
times = Tmax*lengths/sum(lengths);
%steps(k) è il tempo totale trascorso fino al tratto k
steps = zeros(1,length(times));
for k=1:length(steps)
    steps(k) = sum(times(1:k));
end
steps(end) = Tmax;
%caso in cui si sceglie un tempo di percorrenza fissato indipendentemente
%dalla lunghezza del tratto k
% step = T/(size(points,1)-1); 

%%
x_star = @(t)(trajToTimeFunc_x(t, traj_points, [], steps, 0));
y_star = @(t)(trajToTimeFunc_y(t, traj_points, [], steps, 0));
xd_star = @(t)(trajToTimeFunc_x(t, traj_points, [], steps, 1));
yd_star = @(t)(trajToTimeFunc_y(t, traj_points, [], steps, 1));
theta_star = @(t)(atan2(yd_star(t),xd_star(t)));

%%
%MOTO PUNTO-PUNTO
x0 = traj_points(1,1);
y0 = traj_points(1,2);
theta0 = pi/2;
w_alpha = 0.2; %rad/s

v = [];
w = [];
X=[x0; y0; theta0];
for k=1:size(traj_points,1)-1
    %calcolo l'angolo a cui devo trovarmi prima di traslare
    theta_traj = atan2((traj_points(k+1,2)-X(2,end)),(traj_points(k+1,1)-X(1,end)));
    %ruoto finchè non mi giro in quella direzione
    diff_angle = atan2(sin(theta_traj-X(3,end)),cos(theta_traj-X(3,end)));
    while(X(3,end)~=theta_traj)
        Xdot = [0; 0; diff_angle*w_alpha];
        X(:,end+1) = X(:,end) + Xdot;
        if(X(3,end)>pi)
            X(3,end) = -(pi-(X(3,end)-pi));
        end
        if(abs(X(3,end)-theta_traj)<1e-3) %sempre problemi con i double
            X(3,end) = theta_traj;
        end
        v = [v, 0];
        w = [w, Xdot(3)];
    end
    %traslo fino al prossimo punto
    if(k==1)
        v_lin = sqrt(xd_star(0:tc:steps(k)).^2 + yd_star(0:tc:steps(k)).^2);
    else
        v_lin = sqrt(xd_star(steps(k-1):tc:steps(k)).^2 + yd_star(steps(k-1):tc:steps(k)).^2);
    end
    v = [v, v_lin];
    w = [w, zeros(1,length(v_lin))];
    for t=1:length(v_lin)
        Xdot = [v_lin(t)*cos(X(3,end)); v_lin(t)*sin(X(3,end)); 0];
        X(:,end+1) = X(:,end) + tc*Xdot;
    end
end

%plot robot
h = figure(); hold on; grid on;
axis([0 max(traj_points(:,1))+1 0 max(traj_points(:,2))+1]);
plot(traj_points(:,1), traj_points(:,2), 'b', 'LineWidth',2);
for k=1:size(X,2)
    axis('equal');
    plot_robot_traj(X,k,2,[],h);
    pause(0.005)
end 

%MODELLO DD
r = 0.05; %raggio ruote
L = 0.15; %distanza tra le ruote

%matrice che mi permette di passare dalla velocità di rotazione delle ruote
%a velocità di traslazione e rotazione
K_wRwL_to_vw = [
    r/2 r/2;
    r/L -r/L
    ];

K_vw_to_wRwL = inv(K_wRwL_to_vw);
wRwL = K_vw_to_wRwL * [v(:)';w(:)'];
wR = wRwL(1,:); %velocità angolare ruota destra, vR = wR*r
wL = wRwL(2,:); %velocità angolare ruota sinistra, vL = wL*r

t = 0:length(v)-1;
f = figure();
f.WindowState = 'maximized'; hold on; grid on;
subplot(2,3,[1 4]); axis equal;
plot(X(1,:), X(2,:), 'b', 'LineWidth',2); title('Traiettoria percorsa');
subplot(232); hold on; grid on; plot(t,w);title('w(t)');
subplot(235); hold on; grid on; plot(t,v);title('v(t)');
subplot(233); hold on; grid on; plot(t,wR);title('wR');
subplot(236); hold on; grid on; plot(t,wL);title('wL');
end