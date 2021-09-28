%%
clear all;
close all;
warning('off','all');
%%
%CONFIGURAZIONE MAPPA
nrows = 5;
ncols = 6;
%imposto punti di start e goal
start = [0.8,4.2];
goal=[4.6,0.7];
infos = [];
trajectory = [];

%%
%TECNICHE DI PROGETTAZIONE DELLA TRAIETTORIA
%uncommentare quella scelta

%POTENZIALI ARTIFICIALI
%trajectory = gen2_APF(start,goal,nrows,ncols);

%GRAFO DI VISIBILITA'
trajectory = gen1_VG(start,goal,nrows,ncols);

%DIAGRAMMI DI VORONOI
%trajectory = gen3_VOR(start,goal,nrows,ncols);

%POTENZIALI ARTIFICIALI DISCRETI
%trajectory = gen4_APFD(start,goal,nrows,ncols);

%versione smooth della traiettoria scelta
%da non usare con APF, è già una traiettoria smooth
%con Voronoi ci vuole più tempo, sono tanti punti
if(false) %se non si vuole usare la traiettoria smooth, mettere false qui
    [infos, trajectory] = smooth_trj(trajectory);
    %plot smooth_trj
    figure; hold on;
    plot(trajectory(:,1),trajectory(:,2),'r.');
    for k=1:size(infos,1)
        p1 = trajectory(k,:);
        p2 = trajectory(k+1,:);
        if(infos(k,4)=='l')
            plot([p1(1),p2(1)],[p1(2),p2(2)],'k');
        else
            cx = infos(k,1);
            cy = infos(k,2);
            r = infos(k,3);
            arg_cos0 = (p1(1)-cx)/r;
            arg_sin0 = (p1(2)-cy)/r;
            arg_cos1 = (p2(1)-cx)/r;
            arg_sin1 = (p2(2)-cy)/r;
            [alpha0,alpha1] = angles_between(arg_cos0, arg_sin0, arg_cos1, arg_sin1);
            if(alpha0>alpha1)
                dir = -1;
            else
                dir = 1;
            end
            angle = alpha0:0.01*dir:alpha1+0.01*dir;
            if(angle(end)~=alpha1)
                angle(end) = alpha1;
            end
            plot(cx+r*cos(angle),cy+r*sin(angle));
        end
    end
    waitforbuttonpress;
end
%%
%MOTO PUNTO-PUNTO
%usare solo con traiettorie formate effettivamente da spezzate (grafo visibilità e APFD)
X = puntoPunto(trajectory);

%%
%TRAJECTORY TRACKING - LEGGI DI CONTROLLO 
%lo start del robot si trova in un intorno circolare di raggio 1 attorno al punto di 
%start della traiettoria [-0.5,0.5]. Io assegno l'estremo sinistro a x0 e
%l'estremo destro a y0
%model = traj_tracking(trajectory, infos, -0.5, 0.5, 0.5);

%%
%CONFIGURAZIONI CHE IL CONTROLLORE RIESCE A CONTROLLARE
%PER OGNI TRAIETTORIA SMOOTH ED OGNI LEGGE
%1. Potenziali artificiali (non smooth):
%   - Lineare: v e w non sono costanti
%   - Non lineare: ex=-0.5; ey=0.2; etheta=45
%   - Feedback l.: qualsiasi
%2. Grafo di visibilità:
%   - Lineare: v e w non sono costanti
%   - Non lineare: ex=-0.5; ey=0.1; etheta=90
%   - Feedback l.: qualsiasi
%3. Diagramma di Voronoi:
%   - Lineare: v e w non sono costanti
%   - Non lineare: ex=-0.2; ey=0.5; etheta=-95
%   - Feedback l.: qualsiasi
%4. Potenziali artificiali discreti:
%   - Lineare: v e w non sono costanti
%   - Non lineare: ex=-0.2; ey=0.2; etheta=45
%   - Feedback l.: qualsiasi

