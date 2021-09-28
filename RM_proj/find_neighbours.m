function [neighbours, neigh_vals] = find_neighbours(cell, env_matrix, seen_cells)
%metto i vicini della cella considerata solo se il loro valore è minore
%o uguale a quello corrente
neighbours = zeros(0,2);
neigh_vals = [];
if(cell(2)-1 >= 1 && ... %[i,j-1]
        env_matrix(cell(1),cell(2)-1)<=env_matrix(cell(1),cell(2)) && ...
        size(find(sum(seen_cells==[cell(1),cell(2)-1],2)==2),1)<1) 
    neighbours = [neighbours; cell(1),cell(2)-1];
    neigh_vals(end+1) = env_matrix(cell(1),cell(2)-1);
end
if(cell(1)+1 <= size(env_matrix,1) && ...%[i,j+1]
        env_matrix(cell(1),cell(2)+1)<=env_matrix(cell(1),cell(2)) && ...
        size(find(sum(seen_cells==[cell(1),cell(2)+1],2)==2),1)<1) 
    neighbours = [neighbours; cell(1),cell(2)+1];
    neigh_vals(end+1) = env_matrix(cell(1),cell(2)+1);
end
if(cell(1)-1 >= 1) %vicini con coordinata i-1
    %se la cella è già stata visitata la salto; per verificare
    %questa condizione, poichè seen_cells è una matrice kx2 dove le
    %colonne rappresentano la i e la j, controllo se esiste una
    %cella con i e j uguali a quella considerata
    if(env_matrix(cell(1)-1,cell(2))<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)-1,cell(2)],2)==2),1)<1) %[i-1,j]
        neighbours = [neighbours; cell(1)-1,cell(2)];
        neigh_vals(end+1) = env_matrix(cell(1)-1,cell(2));
    end
    if(cell(2)-1 >= 1 && ... %[i-1,j-1]
            env_matrix(cell(1)-1,cell(2)-1)<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)-1,cell(2)-1],2)==2),1)<1)
        neighbours = [neighbours; cell(1)-1,cell(2)-1];
        neigh_vals(end+1) = env_matrix(cell(1)-1,cell(2)-1);
    end
    if(cell(2)+1 <= size(env_matrix,2) && ... %[i-1,j+1]
            env_matrix(cell(1)-1,cell(2)+1)<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)-1,cell(2)+1],2)==2),1)<1)
        neighbours = [neighbours; cell(1)-1,cell(2)+1];
        neigh_vals(end+1) = env_matrix(cell(1)-1,cell(2)+1);
    end
end
if(cell(1)+1 <= size(env_matrix,1)) %vicini con coordinata i+1
    if(env_matrix(cell(1)+1,cell(2))<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)+1,cell(2)],2)==2),1)<1)%[i+1,j]
        neighbours = [neighbours; cell(1)+1,cell(2)];
        neigh_vals(end+1) = env_matrix(cell(1)+1,cell(2));
    end
    if(cell(2)-1 >= 1 && ...
            env_matrix(cell(1)+1,cell(2)-1)<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)+1,cell(2)-1],2)==2),1)<1)%[i+1,j-1]
        neighbours = [neighbours; cell(1)+1,cell(2)-1];
        neigh_vals(end+1) = env_matrix(cell(1)+1,cell(2)-1);
    end
    if(cell(2)+1 <= size(env_matrix,2) && ...
            env_matrix(cell(1)+1,cell(2)+1)<=env_matrix(cell(1),cell(2)) && ...
            size(find(sum(seen_cells==[cell(1)+1,cell(2)+1],2)==2),1)<1)%[i+1,j+1]
        neighbours = [neighbours; cell(1)+1,cell(2)+1];
        neigh_vals(end+1) = env_matrix(cell(1)+1,cell(2)+1);
    end
end
end