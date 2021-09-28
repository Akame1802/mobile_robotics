function voronoi_trj = Voronoi_prj(start, goal, data, obsts_4verts, dims, map_plot)
%data la mappa dell'ambiente, questa funzione restituisce una traiettoria
%che è il cammino minimo del grafo costruito a partire dal diagramma di
%Voronoi
%dati in input: start, goal,contorni discretizzati degli ostacoli, vertici ostacoli
%ingrossati,[ncols, nrows], plot della mappa originale

%clono la mappa iniziale in altre figures
h = get(map_plot, 'Children');
voronoi1 = figure(2); voronoi2 = figure(3); 
copyobj(h,voronoi1);copyobj(h,voronoi2);

%creazione diagramma voronoi su mappa
figure(2); hold on;
voronoi(data(:,1),data(:,2));
title("Voronoi diagram: phase 1");
waitforbuttonpress;

%pulizia diagramma voronoi
%devo rimuovere le linee interne agli ostacoli
[vx, vy] = voronoi(data(:,1),data(:,2));
n = size(obsts_4verts, 1);

vx_new = zeros(2,0);
vy_new = zeros(2,0);

for i=1:size(vx,2)
    isinternal = false;
    for k=1:4:n
        p0 = obsts_4verts(k,:);
        p1 = obsts_4verts(k+1,:);
        p3 = obsts_4verts(k+3,:);
        %condizioni intersezione
        if((vx(1,i)>=p1(1) && vx(1,i)<=p3(1) && ...
           vy(1,i)>=p1(2) && vy(1,i)<=p0(2)) || ...
           (vx(2,i)>=p1(1) && vx(2,i)<=p3(1) && ...
           vy(2,i)>=p1(2) && vy(2,i)<=p0(2)))
           isinternal = true;
           break;
        end
    end
    if(~isinternal)
        %queste curve vanno bene
        vx_new = [vx_new(1,:), vx(1,i); vx_new(2,:), vx(2,i)];
        vy_new = [vy_new(1,:), vy(1,i); vy_new(2,:), vy(2,i)];
    end
end

%a questo punto vanno tolte anche le linee esterne alle figure in modo che
%rimangano solo quelle che definiscono gli archi del grafo
tmp_vx = zeros(2,0);
tmp_vy = zeros(2,0);
for i=1:size(vx_new,2)
    removable = false;
    if((vx_new(1,i)==0 && vy_new(1,i)~=0 && vy_new(1,i)~=dims(2)) || ...
       (vx_new(2,i)==0 && vy_new(2,i)~=0 && vy_new(2,i)~=dims(2)))
        removable = true;
    elseif((vx_new(1,i)~=0 && vy_new(1,i)==0 && vx_new(1,i)~=dims(1)) || ...
       (vx_new(2,i)~=0 && vy_new(2,i)==0 && vx_new(2,i)~=dims(1)))
        removable = true;
    elseif((vx_new(1,i)==dims(1) && vy_new(1,i)~=0 && vy_new(1,i)~=dims(2)) || ...
       (vx_new(2,i)==dims(1) && vy_new(2,i)~=0 && vy_new(2,i)~=dims(2)))
        removable = true;
    elseif((vx_new(1,i)~=0 && vy_new(1,i)==dims(2) && vx_new(1,i)~=dims(1)) || ...
       (vx_new(2,i)~=0 && vy_new(2,i)==dims(2) && vx_new(2,i)~=dims(1)))
        removable = true;
    elseif(vx_new(1,i)<0 || vx_new(1,i)>dims(1) || vy_new(1,i)>dims(2) || vy_new(1,i)<0||...
           vx_new(2,i)<0 || vx_new(2,i)>dims(1) || vy_new(2,i)>dims(2) || vy_new(2,i)<0)
        removable = true;
    end
    if(~removable)
        tmp_vx = [tmp_vx(1,:), vx_new(1,i); tmp_vx(2,:), vx_new(2,i)];
        tmp_vy = [tmp_vy(1,:), vy_new(1,i); tmp_vy(2,:), vy_new(2,i)];
    end
end

vx_new = tmp_vx;
vy_new = tmp_vy;

%ora devo trovare i nodi a partire dai quali va costruito il grafo
%cambio la formattazione dei punti all'interno delle matrici vx_new,
%vy_new(Voronoi edges)
tmp_x = [vx_new(1,:), vx_new(2,:)];
tmp_y = [vy_new(1,:), vy_new(2,:)];
%trovo i punti unici all'interno di questa matrice della forma nx2
%round serve perchè altrimenti non riconosce quei punti come unici (essendo double)
uniques = unique(round(transpose([tmp_x; tmp_y]),3),'rows');
%ritrazione dei punti start e goal sul diagramma di Voronoi
minDistS = Inf; minIndS = -1; minDistS2 = Inf; minIndS2 = -1;
minDistG = Inf; minIndG = -1; minDistG2 = Inf; minIndG2 = -1;
for i=1:size(uniques,1)
    d1 = pdist([start; uniques(i,:)], 'Euclidean');
    d2 = pdist([goal; uniques(i,:)], 'Euclidean');
    if(d1 < minDistS)
        minDistS2 = minDistS;
        minIndS2 = minIndS;
        minDistS = d1;
        minIndS = i;
    end
    if(d2 < minDistG)
        minDistG2 = minDistG;
        minIndG2 = minIndG;
        minDistG = d2;
        minIndG = i;
    end
end

first = minIndS; last = minIndG;
%a questo punto si costruisce il grafo
s = []; %nodi di partenza
t = []; %nodi di arrivo
weights = [];
%controllo se start e/o goal si trovano sulla frontiera del diagramma di
%Voronoi e, se ne fanno parte, aggiungo un arco che va da start/goal ai
%suoi due più prossimi vicini
if(uniques(minIndS,1)==start(1) || uniques(minIndS,2)==start(2))
    first = size(uniques,1)+1; %questa scelta è stata fatta perchè ciascun
    %punto in posizione i in uniques viene indicato nel grado dal nodo i,
    %quindi il primo indice dispobile è size(uniques,1)+1
    s = [s, first, first];
    t = [t, minIndS, minIndS2];
    weights = [weights, minDistS, minDistS2];
end
if(uniques(minIndG,1)==goal(1) || uniques(minIndG,2)==goal(2))
    last = size(uniques,1)+2; %stesso discorso di first
    s = [s, last, last];
    t = [t, minIndG, minIndSG];
    weights = [weights, minDistG, minDistG2];
end
%uso vx_new e vy_new perchè per come sono fatti mantengono l'ordine
%coord_partenza->coord_arrivo
for i=1:size(vx_new,2)
    %trovo l'indice dei punti che sto cercando all'interno di uniques
    %perchè costruisco il grafo sulla base di questi indici
    %n1: nodo di partenza, n2: nodo di arrivo
    tpm = sum(uniques==round([vx_new(1,i), vy_new(1,i)],3),2);
    n1 = find(tpm==2);
    tpm = sum(uniques==round([vx_new(2,i), vy_new(2,i)],3),2);
    n2 = find(tpm==2);
    w = pdist([vx_new(1,i), vy_new(1,i); vx_new(2,i), vy_new(2,i)], 'Euclidean');
    s = [s, n1];
    t = [t, n2];
    weights = [weights, w];
end

G = graph(s,t,weights);
sp = shortestpath(G, first, last);
voronoi_trj = []; %coordinate dei punti della traiettoria
%al grafo vanno aggiunti i nodi di start e goal se non si trovavano già
%sulla frontiera
if(first~=size(uniques,1)+1)
    voronoi_trj = [voronoi_trj, start];
end
for i=1:size(sp,2)
    if(sp(i)<=size(uniques,1))
        voronoi_trj = [voronoi_trj; uniques(sp(i),:)];
    end
end
if(last~=size(uniques,1)+2)
    voronoi_trj = [voronoi_trj; goal];
end

%diagramma di Voronoi ripulito
figure(3);hold on;
plot(vx_new, vy_new, 'b');
%plot ritrazione start e goal su diagramma di Voronoi, se necessario
if(first~=size(uniques,1)+1)
    plot(uniques(minIndS,1),uniques(minIndS,2),'g.','Markersize',20);
    plot([start(1),uniques(minIndS,1)], [start(2),uniques(minIndS,2)], 'g-', 'LineWidth', 1);
else
    plot([start(1),uniques(sp(2),1)], [start(2),uniques(sp(2),2)], 'g-', 'LineWidth', 1);
end
if(last~=size(uniques,1)+2)
    plot(uniques(minIndG,1),uniques(minIndG,2),'g.','Markersize',20);
    plot([goal(1),uniques(minIndG,1)], [goal(2),uniques(minIndG,2)], 'g-', 'LineWidth', 1);
else
    plot([goal(1),uniques(sp(end-1),1)], [goal(2),uniques(sp(end-1),2)], 'g-', 'LineWidth', 1);
end
%plot shortest path
for node=1:size(sp, 2)-1
    if(sp(node)<=size(uniques,1))
        p1 = uniques(sp(node),:);
        p2 = uniques(sp(node+1),:);
        plot([p1(1),p2(1)], [p1(2),p2(2)], 'g-', 'LineWidth', 1);
    end
end
plot([uniques(minIndG,1),goal(1)], [uniques(minIndG,2),goal(2)], 'g-', 'LineWidth', 1);
title("Voronoi diagram: phase 2");
waitforbuttonpress;
end