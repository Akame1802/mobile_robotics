function [hasSolution, shortestPath] = backtrackingSP(cell, goal, delta, env_matrix, seen_cells, path)
%condizione di uscita
if(env_matrix(cell(1),cell(2))==0) %se arrivo al goal ho finito
    hasSolution = true;
    shortestPath = path;
    return;
end
[neighbours, neigh_vals] = find_neighbours(cell, env_matrix, seen_cells);
if(size(neighbours,1)==0)%non c'è un percorso che porta al goal (o non viene trovato in questo ramo)
    hasSolution = false;
    shortestPath = path;
    return;
end

[out, idx] = sort(neigh_vals); %ordino i vicini in base al loro valore
%a parità di valore, i vicini vengono scelti in base alla loro distanza
%euclidea dal goal (precedenza a chi ha distanza minore)
i = 1;
while(i<=length(idx))
    j = i+1;
    while(j<=length(idx))
        if(neigh_vals(idx(i))~=neigh_vals(idx(j)))
            break; %perchè devo indagare sui vicini dal valore uguale
        end
        j = j+1;
    end
    %riporta l'indice sull'ultimo elemento uguale
    j = j-1;
    dists = zeros(1,j-(i-1));
    for k=i:j
        %neighbours(idx(k),2) è la j di questo vicino
        x_c = neighbours(idx(k),2)*delta-delta+delta/2; 
        %neighbours(idx(k),1) è la i di questo vicino
        y_c = (size(env_matrix,1)+1-neighbours(idx(k),1))*delta-delta+delta/2;
        dists(k-(i-1)) = pdist([x_c,y_c; goal],'Euclidean');
    end
    %idx2 sono gli indici del sotto-array che contiene i vicini di valore
    %uguale; li ordino in base alla distanza (crescente) dal goal 
    [out, idx2] = sort(dists);
    %a questo punto devo ridimensionare gli indici idx2 per riottenere gli
    %indici corrispondenti in idx e li sostituisco
    idx2 = idx2 + (i-1);
    idx(i:j) = idx(idx2);
    i = j+1;
end

for i=1:length(idx)
    [hasSolution, shortestPath] = backtrackingSP(neighbours(idx(i),:), goal, delta, ...
        env_matrix, [seen_cells; cell], path);
    if(hasSolution)
        shortestPath = [cell; shortestPath];
        return;
    end
end
end