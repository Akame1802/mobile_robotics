function traj_apfd = APFD_prj(start, goal, obstacles, dims, map_plot)

%creo la matrice d'ambiente dividendo la mappa in celle
delta = 0.5;
n = dims(1)/delta; m = dims(2)/delta;
env_matrix = zeros(n, m);
%come individuare gli indici delle celle della matrice nxm in base alle
%coordinate dei punti start e goal:
%i_mat = n+1-i_plot, dove i_plot=ceil(y/delta)
%j_mat = ceil(x/delta)
i_s = n+1-ceil(start(2)/delta);
j_s = ceil(start(1)/delta);
i_g = n+1-ceil(goal(2)/delta);
j_g = ceil(goal(1)/delta);
%nella matrice, se si ipotizza di inserire un valore crescente diverso in
%ogni cella partendo da 0, il valore massimo che si può inserire è nxm-1;
%di conseguenza, questo valore rappresenta "l'infinito" in questo contesto
inf = n*m+1;

h = get(map_plot, 'Children');
f2 = figure(2);
copyobj(h,f2);
%griglia con ostacoli originali
figure(2); hold on;
title("Cells division");
x = 0:delta:dims(2);
y = 0:delta:dims(1);
for i=1:length(x)
   plot([x(i) x(i)],[y(1) y(end)],'k-') %y grid lines
   hold on    
end
for i=1:length(y)
   plot([x(1) x(end)],[y(i) y(i)],'k-') %x grid lines
   hold on    
end
waitforbuttonpress;

%plot matrice d'ambiente
f3 = figure(3); tiledlayout(1,2); nexttile;
hold on; f3.WindowState = 'maximized';
title("Environment Matrix");
for i=1:length(x)
   plot([x(i) x(i)],[y(1) y(end)],'k-') %y grid lines
   hold on    
end
for i=1:length(y)
   plot([x(1) x(end)],[y(i) y(i)],'k-') %x grid lines
   hold on    
end
%plot celle start e goal
s = rectangle('Position',[j_s*delta-delta, ceil(start(2)/delta)*delta-delta, delta, delta]);
s.FaceColor = 'red';
g = rectangle('Position',[j_g*delta-delta, ceil(goal(2)/delta)*delta-delta, delta, delta]);
g.FaceColor = 'green';
%gestisco la creazione del vicinato e l'assegnamento di valori per "cerchi
%concentrici" con una coda FIFO
queue = zeros(inf,2);
queue(1,:) = [i_g, j_g];
len = 1;
while(len > 0)
    %estraggo l'elemento in testa alla coda di coordinate i,j ed il suo
    %valore nella matrice d'ambiente
    elem = queue(1,:);
    val = env_matrix(elem(1),elem(2));
    text(elem(2)*delta-delta/2-0.06,(n+1-elem(1))*delta-delta/2,int2str(val),'Fontsize',15);
    %avendo rimosso un elemento dalla coda, traslo i successivi di una
    %posizione indietro ed aggiorno la lunghezza della coda
    queue(1:len-1,:) = queue(2:len,:);
    len = len-1;
    %assegno il valore val+1 nella matrice d'ambiente ai vicini a cui è
    %possibile assegnare un valore
    if(elem(1)-1 >= 1) %vicini con coordinata i-1 o i
        if(env_matrix(elem(1)-1,elem(2))==0)
            env_matrix(elem(1)-1,elem(2)) = val+1;
            %inserisco nella coda il nuovo elemento ed aggiorno la lunghezza
            queue(len+1,:) = [elem(1)-1,elem(2)];
            len = len+1;
        end
        if(elem(2)-1 >= 1) %[i-1,j-1]
            if(env_matrix(elem(1)-1,elem(2)-1)==0)
                env_matrix(elem(1)-1,elem(2)-1) = val+1;
                queue(len+1,:) = [elem(1)-1,elem(2)-1];
                len = len+1;
            end
            if(env_matrix(elem(1),elem(2)-1)==0) %[i,j-1]
                env_matrix(elem(1),elem(2)-1) = val+1;
                queue(len+1,:) = [elem(1),elem(2)-1];
                len = len+1;
            end
        end
        if(elem(2)+1 <= m)%[i-1,j+1]
            if(env_matrix(elem(1)-1,elem(2)+1)==0)
                env_matrix(elem(1)-1,elem(2)+1) = val+1;
                queue(len+1,:) = [elem(1)-1,elem(2)+1];
                len = len+1;
            end
            if(env_matrix(elem(1),elem(2)+1)==0) %[i,j+1]
                env_matrix(elem(1),elem(2)+1) = val+1;
                queue(len+1,:) = [elem(1),elem(2)+1];
                len = len+1;
            end
        end
    end
    if(elem(1)+1<=n) %vicini con coordinata i+1
        if(env_matrix(elem(1)+1,elem(2))==0)%[i+1,j]
            env_matrix(elem(1)+1,elem(2)) = val+1;
            queue(len+1,:) = [elem(1)+1,elem(2)];
            len = len+1;
        end
        if(elem(2)-1 >= 1 && env_matrix(elem(1)+1,elem(2)-1)==0)%[i+1,j-1]
            env_matrix(elem(1)+1,elem(2)-1) = val+1;
            queue(len+1,:) = [elem(1)+1,elem(2)-1];
            len = len+1;
        end
        if(elem(2)+1 <= m && env_matrix(elem(1)+1,elem(2)+1)==0)%[i+1,j+1]
            env_matrix(elem(1)+1,elem(2)+1) = val+1;
            queue(len+1,:) = [elem(1)+1,elem(2)+1];
            len = len+1;
        end
    end
    %faccio in modo che nessun vicino del goal possa più modificarlo
    if(elem(1)==i_g && elem(2)==j_g)
        env_matrix(i_g, j_g) = -1;
    end
end
%ripristino il valore della cella relativa al goal
env_matrix(i_g, j_g) = 0;
%inserimento ostacoli in env_matrix e nel plot
for k=1:4:size(obstacles,1)
    %modifica nella matrice
    i_first = max(1, n+1-ceil(obstacles(k,2)/delta));
    j_first = max(1, ceil(obstacles(k,1)/delta));
    i_last = min(n, n+1-ceil(obstacles(k+2,2)/delta));
    j_last = min(m, ceil(obstacles(k+1,1)/delta));
    env_matrix(i_first:i_last,j_first:j_last) = inf;
    %modifica nel plot
    for i=i_first:i_last
        i_plot = n+1-i;
        y_draw = i_plot*delta-delta;
        if(y_draw>=0 && y_draw<=dims(1))
            for j=j_first:j_last
                x_draw = j*delta-delta;
                if(x_draw>=0 && x_draw<=dims(2))
                    o = rectangle('Position',[x_draw, y_draw, delta, delta]);
                    o.FaceColor = 'black';
                    text(x_draw+delta/2-0.06,y_draw+delta/2,'∞','Color','w','Fontsize',15);
                end
            end
        end
    end
end

%calcolo shortest path
path_cells = zeros(0,2);
seen_cells = zeros(0,2);
[hasSolution, path_cells] = backtrackingSP([i_s,j_s], goal, delta, env_matrix, seen_cells, path_cells);

%soluzione senza backtracking
% cell = [i_s,j_s];
% while(env_matrix(cell(1),cell(2))~=0)
%     path_cells = [path_cells; cell];
%     seen_cells = [seen_cells; cell];
%     [neighbours,neigh_vals] = find_neighbours(cell, env_matrix, seen_cells);
%     mins = find(neigh_vals==min(neigh_vals)); %trovo gli indici dei valori di interesse in neigh_vals
%     if(size(mins,2)==0)%non c'è un percorso che porta al goal (o non viene trovato)
%         hasSolution = false;
%         break
%     end
%     minInd = 1;
%     if(size(mins,2)>1)%c'è più di un minimo locale
%         %scelgo la cella che minimizza la distanza euclidea dal goal
%         minDist = Inf;
%         for c=1:size(mins,2)
%             x_c = neighbours(mins(c),2)*delta-delta+delta/2; %neighbours(mins(c),2) è la j di questo vicino
%             y_c = (n+1-neighbours(mins(c),1))*delta-delta+delta/2; %neighbours(mins(c),1) è la i di questo vicino
%             d = pdist([x_c,y_c;goal],'Euclidean');
%             if(d<minDist)
%                 minDist = d;
%                 minInd = c;
%             end
%         end
%     end
%     %in ogni caso vado sulla cella indicata da mins(minInd)
%     cell = neighbours(mins(minInd),:);
% end

last = size(path_cells,1);
%includo la cella del goal nella soluzione
if(hasSolution)
    path_cells = [path_cells; [i_g,j_g]];
    last = size(path_cells,1)-1;
end
%coloro il path minimo e lo converto in coordinate x-y
traj_apfd = [start];
for c=2:last
    drawnow;
    pause(0.8)
    x = path_cells(c,2)*delta-delta;
    y = (n+1-path_cells(c,1))*delta-delta;
    q = rectangle('Position',[x, y, delta, delta]);
    q.FaceColor = 'cyan';
    %punti della traiettoria
    traj_apfd = [traj_apfd;x+delta/2,y+delta/2];
end
if(hasSolution)
    traj_apfd = [traj_apfd; goal];
    %plot traiettoria
    figure(3);nexttile; hold on;
    title('Trajectory');
    axis([0 dims(2) 0 dims(1)]);
    plot(goal(1),goal(2),'g.','MarkerSize',20);
    text(goal(1)+0.2,goal(2),'GOAL');
    plot(start(1),start(2),'r.','MarkerSize',20);
    text(start(1)-0.3,start(2)+0.2,'START');
    plot(traj_apfd(:,1),traj_apfd(:,2),'b-','MarkerSize',20);
else
    traj_apfd = [];
end

waitforbuttonpress;

end