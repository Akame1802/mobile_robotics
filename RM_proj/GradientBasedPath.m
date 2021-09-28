function path = GradientBasedPath(f,start,goal,iterations)
% Questa funzione pianifica la traiettoria di un robot basata sulla direzione del gradiente 
%della funzione in input (ambiente 2D). L'output restituisce una
%traiettoria sotto forma di matrice nx2 di punti definiti dalle coordinate (x,y).

[gx,gy] = gradient(-f);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path = start;
waypoint = start;
step = 3.5; %arbitrario: non deve essere troppo alto
%il valore di step potrebbe non andare bene se si modificano anche di poco
%le coordinate di start e goal, perchè si verificherà un fenomeno di "balancing"
%nell'intorno di goal (che terminerà una volta eseguite tutte le iterazioni)
tolerance = 1; %grado di tolleranza in base al quale decido se sono arrivata a destinazione o meno
while(iterations>0)
    if(norm(goal - waypoint) < tolerance)
        break; %sono arrivata al goal
    end
    dx = gx(floor(waypoint(2)),floor(waypoint(1)));
    dy = gy(floor(waypoint(2)),floor(waypoint(1)));
    
    delta = [dx,dy];
    
    %versori
    dx_direction = dx/norm(delta);
    dy_direction = dy/norm(delta);
    
    %new waypoints
    new_x = waypoint(1)+step*dx_direction;
    new_y = waypoint(2)+step*dy_direction;
    
    waypoint = [new_x,new_y];
    path = [path;waypoint];
    iterations = iterations-1;
end
end
