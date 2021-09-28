function trj = gen2_APF(start,goal,nrows,ncols)
%SCHEMA 2D DELLA MAPPA COME BWIMAGE 
%Ho utilizzato questa tecnica per conoscere la distanza del robot dagli ostacoli 
%attraverso la funzione bwdist, utilizzata esclusivamente nella routine "APF_prj". Per
%questo motivo, sono dovuta ricorrere ad un arteficio grafico riscalando
%l'intera mappa dell'ambiente (in pratica aumentando la risoluzione dell'immagine)

ncols_b = ncols*100;
nrows_b = nrows*100;
[xm,ym]= meshgrid(1:ncols_b,1:nrows_b);

%imposto punti di start e goal
start_b = [start(1)*100,start(2)*100];
goal_b = [goal(1)*100,goal(2)*100];

obstacle = false(nrows_b,ncols_b); %true->ostacolo, false->nessun ostacolo

%ostacoli rettangolari
obstacle(300:450,150:200) = true;
obstacle(180:300,400:550) = true;
%ostacolo quadrato
obstacle(50:150,80:180)=true;
%ostacoli circolari
circle1 = ((xm-180).^2+(ym-50).^2) < 50.^2;
obstacle(circle1) = true;
circle2 = ((xm-400).^2+(ym-300).^2) < 80.^2;
obstacle(circle2) = true;

figure;
imshow(~obstacle, 'InitialMagnification', 'fit'); %ostacoli neri, spazio libero bianco

hold on;
plot(goal_b(1),goal_b(2),'g.','MarkerSize',20);
text(goal_b(1)+15,goal_b(2),'GOAL');
plot(start_b(1),start_b(2),'r.','MarkerSize',20);
text(start_b(1)-30,start_b(2)+20,'START');

axis([0 ncols_b 0 nrows_b]);
axis xy; axis on;
xlabel('x'); ylabel('y');
xticklabels(0:ncols_b/100);
yticklabels(0:0.5:nrows_b/100);
title('Map BWImage');
waitforbuttonpress;

%POTENZIALI ARTIFICIALI (fare riferimento allo schema 2d della mappa come bwimage)
trj = APF_prj(start_b,goal_b,obstacle, xm, ym)/100;
end