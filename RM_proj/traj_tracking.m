function [f_robot] = traj_tracking(traj_points, infos, error_x, error_y, error_theta)

%LEGGE DI CONTROLLO 1 (LINEARIZED CONTROL): si ottiene linearizzando un sistema non lineare
%attorno all'errore nullo (punto di equilibrio). Garantisce la stabilità
%locale solo per v* e w* costanti.
%LEGGE DI CONTROLLO 2(NON-LINEAR CONTROL): legge di controllo per sistemi non lineari. v* e w*
%sono limitate con derivate limitate e non devono convergere contemporaneamente a
%zero e poichè e->0 per t->inf, questa legge è globalmente stabilizzante
%LEGGE DI CONTROLLO 3 (FEEDBACK LINEARIZATION): non ho problemi se v* e w*
%convergono contemporaneamente a zero, il punto b scelto seguirà
%asintoticamente la traiettoria stabilita.
f_robot = [];

%%
tc = 0.1; %tempo di campionamento
%npoints/Tmax = tc
Tmax = min(50,(size(traj_points,1)-1)/tc);

%calcolo dei tempi di percorrenza di ciascun tratto
lengths = zeros(1, size(traj_points,1)-1); %lunghezze dei tratti percorsi
for k=1:length(lengths)
    p1 = traj_points(k,:);
    p2 = traj_points(k+1,:);
    if(isempty(infos) || infos(k,4)=='l')%se il tratto è lineare, la lunghezza è calcolata come dist. euclidea
        lengths(k) = pdist([p1;p2],'Euclidean');
    else %se il tratto è circolare, calcolo la lunghezza dell'arco di circonferenza (l_arco = angolo_rad spazzato*raggio)
        cx = infos(k,1);
        cy = infos(k,2);
        r = infos(k,3);
        arg_cos0 = (p1(1)-cx)/r;
        arg_sin0 = (p1(2)-cy)/r;
        arg_cos1 = (p2(1)-cx)/r;
        arg_sin1 = (p2(2)-cy)/r;
        [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1);
        lengths(k) = abs(alpha1-alpha0)*r;
    end
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
x_star = @(t)(trajToTimeFunc_x(t, traj_points, infos, steps, 0));
y_star = @(t)(trajToTimeFunc_y(t, traj_points, infos, steps, 0));
xd_star = @(t)(trajToTimeFunc_x(t, traj_points, infos, steps, 1));
yd_star = @(t)(trajToTimeFunc_y(t, traj_points, infos, steps, 1));
xdd_star = @(t)(trajToTimeFunc_x(t, traj_points, infos, steps, 2));
ydd_star = @(t)(trajToTimeFunc_y(t, traj_points, infos, steps, 2));
theta_star = @(t)(atan2(yd_star(t),xd_star(t)));

%%
%leggi di controllo: uncommentare quella scelta
%LINEARIZED CONTROL
%a->pulsazione naturale, delta->coeff. di smorzamento
%maggiore è a, minore sarà il tempo di assestamento al segnale da inseguire
% point_guide = 0;
% a = 1; delta = 0.99; %a>0,0<delta<1
% f_robot = @(t,X)(lin_control(t,X,x_star,y_star,theta_star,...
%                  xd_star,yd_star,xdd_star,ydd_star,a,delta));


%NON-LINEAR CONTROL
% point_guide = 0;
% %funzioni limitate con derivate limitate
% k1 = @(vs,ws)(ones(size(vs)));
% k3 = @(vs,ws)(ones(size(vs)));
% k2 = 1; %k2>0
% f_robot = @(t,X)(NL_control(t,X,x_star,y_star,theta_star,...
%                   xd_star,yd_star,xdd_star,ydd_star,k1,k2,k3));

%FEEDBACK LINEARIZATION
point_guide = 1;
k1=10; k2=10; b=0.1;%asintoticamente il punto B coincide con la traiettoria
f_robot = @(t,X)(FL_control(t,X,x_star,y_star,xd_star,yd_star,k1,k2,b));

%%
%condizioni di partenza
x0 = x_star(0)+error_x;
y0 = y_star(0)+error_y;
if point_guide==0
    theta0 = theta_star(0)+deg2rad(error_theta);
else
    theta0 = 0;
end

%%
[t,evolution] = ode45(f_robot,[0,Tmax],[x0,y0,theta0]); %input: function,tempo,stato_sistema

x = evolution(:,1);
y = evolution(:,2);
theta = evolution(:,3);

%N.B. Pur facendo in modo di mantenere costante il rapporto spazio
%percorso/tempo di percorrenza nel momento in cui si stabiliscono gli
%intervalli di tempo per ogni tratto, le velocità v e w non sono costanti
%perchè il tempo di campionamento utilizzato in ode45 non è costante (qui
%sotto la dimostrazione)
% a = x_star(t);
% dv = zeros(1,length(a)-1);
% for i=1:length(a)-1
%     (a(i+1)-a(i)),t(i+1)
%     dv(i) = (a(i+1)-a(i))/t(i+1);
% end
% return
%%
%sezione plot
f = figure(); f.WindowState = 'maximized';
if point_guide==0
    subplot(3,3,[1,4,7]);
    hold on; grid on;
    plot_robot([x0;y0;theta0],1,f); %effettiva
    plot_robot([x_star(0);y_star(0);theta_star(0)],2,f); %traiettoria da inseguire
end

subplot(332);hold on; grid on;
title('x(t)');
plot(t,x,'LineWidth',2);
plot(t,x_star(t),'r','LineWidth',2);

subplot(335);hold on; grid on;
title('y(t)');
plot(t,y,'LineWidth',2);
plot(t,y_star(t),'r','LineWidth',2);

if point_guide==0
    subplot(338); hold on; grid on;
    title('θ(t)');
    plot(t,theta,'LineWidth',2);
    plot(t,theta_star(t),'r','LineWidth',2);
end

subplot(333); hold on; grid on;
title('ex');
plot(t,x_star(t)-x);
subplot(336); hold on; grid on;
title('ey');
plot(t,y_star(t)-y);
if point_guide==0
    subplot(339); hold on; grid on;
    title('eθ');
    plot(t,rad2deg(atan2(sin(theta_star(t)-theta),cos(theta_star(t)-theta))));
end

%draw
pause();
if point_guide==0
    evolution_star = [x_star(t),y_star(t),theta_star(t)];
end
subplot(3,3,[1,4,7]);
for i = 1:2:size(evolution,1)
    tcur = t(1:i);
    plot_robot_traj(evolution',i,1,[],f);
    title(num2str(i/size(evolution,1)));
    if point_guide==0
        plot_robot_traj(evolution_star',i,2,[],f);
    else
        plot(x_star(tcur),y_star(tcur),'c-','LineWidth',3);
    end
    pause(0.2);
end
waitforbuttonpress;

%%
%plot model DD
[res, v, w] = f_robot(t', [x0,y0,theta0]);

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

f = figure();
f.WindowState = 'maximized'; hold on; grid on;
subplot(2,3,[1 4]); axis equal;
plot(x, y, 'b', 'LineWidth',2); title('Traiettoria percorsa');
subplot(232); hold on; grid on; plot(t,w);title('w(t)');
subplot(235); hold on; grid on; plot(t,v);title('v(t)');
subplot(233); hold on; grid on; plot(t,wR);title('wR');
subplot(236); hold on; grid on; plot(t,wL);title('wL');

end