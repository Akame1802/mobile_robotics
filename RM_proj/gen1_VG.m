function trj = gen1_VG(start,goal,nrows,ncols)

%MAPPA MODIFICATA CON OSTACOLI INGROSSATI 
modmap = figure; 
grid on; tiledlayout(1,2); nexttile;
modmap.WindowState = 'maximized';

%genero gli ostacoli sulla mappa
lq = 1; %lato quadrato
br1 = 0.5; hr1 = 1.5; %dimensioni rettangolo 1
br2 = 1.5; hr2 = 1.2; %dimensioni rettangolo 2
rc1 = 0.5; %raggio circonferenza 1
rc2 = 0.8; %raggio circonferenza 2

delta = 0.2;
%rettangolo 1 
r1 =[1.5,3,br1,hr1];
dr1 = rectangle('Position',r1);
dr1.FaceColor = 'black'; dr1.EdgeColor = 'k';
%rettangolo 1 ingrossato
rg1 =[r1(1)-delta/2,r1(2)-delta/2,br1+delta,hr1+delta];
drg1 = rectangle('Position',rg1);
drg1.EdgeColor = 'r'; drg1.LineStyle = '--';
%quadrato
q =[0.8,0.5,lq,lq];
dq = rectangle('Position',q);
dq.FaceColor = 'black'; dq.EdgeColor = 'k';
%cerchio 1
xc1=1.8; yc1=0.5;
t1=linspace(0,2*pi);
patch(xc1+rc1*cos(t1),yc1+rc1*sin(t1),'k','EdgeColor','k');
%quadrato+cerchio 1 ingrossati
lqg1 = rc1 + lq;
qg1 = [min(q(1), xc1-rc1)-delta/2, min(q(2), yc1-rc1)-delta/2, lqg1+delta, lqg1+delta];
dqg1 = rectangle('Position',qg1);
dqg1.EdgeColor = 'r'; dqg1.LineStyle = '--';
%rettangolo 2
r2 =[4,1.8,br2,hr2];
dr2 = rectangle('Position',r2);
dr2.FaceColor = 'black'; dr2.EdgeColor = 'k';
%cerchio 2
xc2=4; yc2=3;
t2=linspace(0,2*pi);
patch(xc2+rc2*cos(t2),yc2+rc2*sin(t2),'k','EdgeColor','k');
%rettangolo 2+cerchio 2 ingrossati
brg2 = rc2 + br2; hrg2 = rc2 + hr2;
rg2 =[min(r2(1), xc2-rc2)-delta/2, min(r2(2), yc2-rc2)-delta/2, brg2+delta, hrg2+delta];
drg2 = rectangle('Position',rg2);
drg2.EdgeColor = 'r'; drg2.LineStyle = '--';

hold on; 
axis([0 ncols 0 nrows]);
plot(goal(1),goal(2),'g.','MarkerSize',20);
text(goal(1)+0.2,goal(2),'GOAL');
plot(start(1),start(2),'r.','MarkerSize',20);
text(start(1)-0.3,start(2)+0.2,'START');
title('2D Modified Map');
%%

rect1 = [rg1(1),rg1(2)+rg1(4);
         rg1(1),rg1(2);
         rg1(1)+rg1(3),rg1(2)+rg1(4);
         rg1(1)+rg1(3),rg1(2)];
sqr = [qg1(1),qg1(2)+qg1(3);
       qg1(1),0;
       qg1(1)+qg1(3),qg1(2)+qg1(3);
       qg1(1)+qg1(3),0];
rect2 = [rg2(1),rg2(2)+rg2(4);
         rg2(1),rg2(2);
         rg2(1)+rg2(3),rg2(2)+rg2(4);
         rg2(1)+rg2(3),rg2(2)];
bounds = [0,nrows;
          0,0;
          ncols,nrows;
          ncols,0];
      
%nodes contiene tutte le coordinate (x,y) dei nodi del grafo     
nodes = [start;rect1;sqr;rect2;bounds;goal];
%figs_map contiene, all'indice i, l'indice in nodes dell'ultimo punto della
%figura i (il primo punto, invece, è individuato da figs_map(i-1)+1)
figs_map = [1,5,9,13,17,18];

hold on;
plot(nodes(2:end-1,1),nodes(2:end-1,2),'b.','MarkerSize',20);

%GRAFO DI VISIBILITA' (fare riferimento allo schema 2d map modified)
trj = Visibility_Graph(nodes, figs_map, modmap);
end