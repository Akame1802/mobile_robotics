function traj_apf = APF_prj(start,goal,obstacle, x, y)
%Data la configurazione dell'ambiente specificato dalla matrice obstacle, questa funzione
%utilizza la tecnica dei potenziali artificiali per generare una
%traiettoria dal punto di start al punto di goal dati in input. L'output restituisce una
%traiettoria sotto forma di matrice nx2 di punti definiti dalle coordinate (x,y).
%Questa funzione si appoggia alla funzione ausiliaria "GradientBasedPath"

%definisco i potenziali repulsivi
%repulsive(x,y) = eta(1/rho(x)-1/d0)^2 se rho(x)≤ d0, 0 altrimenti

d = bwdist(obstacle); %bwdist restituisce la distanza da qualsiasi valore 'true' nella matrice obstacle
k = 100; %parametro arbitrario
rho = d/k+1; %sommo 1 perchè alcuni valori in d potrebbero essere nulli e così evito di dividere per zero
d0=2; %arbitrario: quando il robot si trova ad una distanza >d0 dall'ostacolo, fr è considerata nulla
eta = 900; %arbitrario: cambiando questo parametro controllo la "grandezza" (strenghtness) della forza repulsiva
            %ad esempio se eta=1 -> max(repulsive) = 0.25 e max(attractive) = 400
            %con un valore alto di eta bilancio le due forze e faccio in modo che il robot eviti certamente gli ostacoli
            
repulsive = eta*((1./rho -1/d0).^2);
repulsive(rho>d0)=0;

%plot repulsive potential
f1 = figure(1); tiledlayout(1,2); nexttile;
f1.WindowState = 'maximized';
m = mesh(repulsive);
axis equal;
title('Repulsive Potential');

%definisco il potenziale attrattivo

ksi=1/700; %arbitrario: se ksi=1/10000 il robot non raggiungerà il goal perchè la forza attrattiva sarà 
           %troppo debole; se invece ksi=1/10, ad esempio, il robot attraverserà gli ostacoli
attractive = ksi*((x-goal(1)).^2+(y-goal(2)).^2);

%plot attractive potential
figure(1); nexttile;
m = mesh(attractive);
axis equal;
title('Attractive Potential');
waitforbuttonpress;

%definisco il potenziale risultante
f = attractive+repulsive;
f2 = figure(2); tiledlayout(1,2); nexttile;
f2.WindowState = 'maximized';
m = mesh(f);
axis equal;
title('Total Potential');

%planning path
traj_apf = GradientBasedPath(f,start,goal,1000); %1000 iterazioni del loop

%animazione
[rx,ry,rz] = sphere();
R = 20;
rx = R*rx; ry = R*ry; rz = R*rz+R; %per rz aggiungo R così vedo la sfera al di sopra della superficie

hold on;
s = mesh(rx,ry,rz); %posizione (0,0,0)
s.FaceColor = 'red';
s.EdgeColor = 'none';

hold on;
plot(goal(1),goal(2),'g*','MarkerSize',25);

for i = 1:size(traj_apf,1)
    P = round(traj_apf(i,:));
    z = f(P(2),P(1)); %z=f(x,y)
    
    s.XData = rx+P(1);
    s.YData = ry+P(2);
    s.ZData = rz+z;
    
    drawnow;
    pause(0.05)
end

%plot trajectory
figure(2); nexttile;
hold on; axis([0 x(end) 0 y(end)]);
axis xy; axis on;
plot(traj_apf(:,1),traj_apf(:,2),'b-','MarkerSize',15);
plot(goal(1),goal(2),'g.','MarkerSize',20);
plot(start(1),start(2),'r.','MarkerSize',20);
title('Trajectory');
waitforbuttonpress; hold off;

end