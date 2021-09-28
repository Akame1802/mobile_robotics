%dati n punti planari smooth_trj restituisce le informazioni necessarie
%(infos) alla definizione delle leggi orarie per una traiettoria lineare con
%curvatura in prossimità di angoli e le coordinate dei punti (new_points)
%che definiscono la traiettoria, inclusi quelli di raccordo tra un segmento e il successivo. 
%La legge oraria per x(t) e y(t) sarà data dalla composizione di più leggi
%orarie: moto rettilineo uniforme per traiettorie lineari e moto armonico
%per traiettorie circolari.

%le tuple restituite in infos sono del tipo:
%[m,q,0,'l'] per le traiettorie lineari
%[Cx,Cy,R,'b'] per le traiettorie curvilinee 
%(il robot deve percorrere l'arco di circonferenza di centro (Cx,Cy) e raggio R)
%'l' e 'b' sono distinti secondo il loro ASCII

function [infos, new_points] = smooth_trj(points)
    syms x y
    %numero di punti iniziale, finale e di raccordo
    n_points = 2 + 2*(length(points)-2);
    %in new_points metto punto iniziale, punto finale e punti di raccordo (no waypoints)
    new_points = [];
    %infos -> all'interno ci sono tutti i parametri utili alla definizione delle leggi orarie
    %per indicare il tipo di traiettoria, le tuple sono identificate dagli ASCII di 'l' e 'b' (linear & blending)
    infos = [];
    prev = -1;
    i = 1;
    %con i indicizzo points, con p indicizzo new_points e infos
    for p=1:n_points-1
        if mod(p,2)~=0 %sto percorrendo una traiettoria lineare
            %il punto di raccordo si trova al 90% della retta corrente e ne
            %trovo le coordinate
            seg1 = pdist([points(i,1),points(i,2);points(i+1,1),points(i+1,2)],'euclidean');
            dist_lin = seg1-seg1*0.1; 
            if prev==-1
                %aggiungo il punto iniziale e parametri in infos
                new_points(end+1,1) = points(i,1);
                new_points(end,2) = points(i,2);
                prev = points(i,:);
            end
            if p<n_points-2
            %in questo punto finisce la traiettoria lineare
                %se si stanno considerando rette del tipo x=k oppure y=k 
                %allora c'è un problema (divisione per 0 nell'equazione 
                %della retta passante per due punti) e queste rette creeranno
                %inconsistenze
                if((points(i+1,1)-points(i,1))~=0 && (points(i+1,2)-points(i,2))~=0)
                    sols = solve(sqrt((x-points(i,1)).^2+(y-points(i,2)).^2)==dist_lin, ...
                        (y-points(i,2))/(points(i+1,2)-points(i,2))==(x-points(i,1))/(points(i+1,1)-points(i,1)),[x, y]);
                elseif((points(i+1,1)-points(i,1))==0 && (points(i+1,2)-points(i,2))~=0)
                    sols = solve(sqrt((x-points(i,1)).^2+(y-points(i,2)).^2)==dist_lin, ...
                                 x==points(i+1,1),[x,y]);
                else
                    sols = solve(sqrt((x-points(i,1)).^2+(y-points(i,2)).^2)==dist_lin, ...
                                 y==points(i+1,2),[x,y]);
                end
                %prendo il punto più vicino al prossimo a distanza dist_lin dal punto corrente
                if pdist([points(i+1,1),points(i+1,2);double(sols.x(1,1)),double(sols.y(1,1))],'euclidean') < ...
                   pdist([points(i+1,1),points(i+1,2);double(sols.x(2,1)),double(sols.y(2,1))],'euclidean')
                    p1 = [double(sols.x(1,1)), double(sols.y(1,1))];
                else
                    p1 = [double(sols.x(2,1)), double(sols.y(2,1))];
                end
            else
                p1 = [points(i+1,1),points(i+1,2)];
            end
            new_points(end+1, 1) = p1(1);
            new_points(end, 2) = p1(2);
            m = (p1(2)-prev(2))/(p1(1)-prev(1));
            q = p1(2)-m*p1(1);
            infos(end+1,1) = m;
            infos(end,2) = q;
            infos(end,4) = 'l';
            %sarà utilizzato come punto da cui iniziare la curvatura
            prev = p1;
        else %mod(p,2)==0: tratto curvilineo
            seg1 = pdist([points(i,1),points(i,2);points(i+1,1),points(i+1,2)],'euclidean');
            dist_lin = seg1-seg1*0.1;
            dist_blend = seg1-dist_lin;
            %trovo il secondo punto di raccordo che giace sul successivo
            %segmento
            %stesso discorso di prima
            if((points(i+2,1)-points(i+1,1))~=0 && (points(i+2,2)-points(i+1,2))~=0)
                sols = solve(sqrt((x-points(i+1,1)).^2+(y-points(i+1,2)).^2)==dist_blend, ...
                    (y-points(i+1,2))/(points(i+2,2)-points(i+1,2))==(x-points(i+1,1))/(points(i+2,1)-points(i+1,1)),[x, y]);
            elseif((points(i+2,1)-points(i+1,1))==0 && (points(i+2,2)-points(i+1,2))~=0)
                sols = solve(sqrt((x-points(i+1,1)).^2+(y-points(i+1,2)).^2)==dist_blend, ...
                             x==points(i+2,1),[x,y]);
            else
                sols = solve(sqrt((x-points(i+1,1)).^2+(y-points(i+1,2)).^2)==dist_blend, ...
                             y==points(i+2,2),[x,y]);
            end
            if pdist([points(i+2,1),points(i+2,2);double(sols.x(1,1)),double(sols.y(1,1))],'euclidean') < ...
               pdist([points(i+2,1),points(i+2,2);double(sols.x(2,1)),double(sols.y(2,1))],'euclidean')
                p2 = [double(sols.x(1,1)), double(sols.y(1,1))];
            else
                p2 = [double(sols.x(2,1)), double(sols.y(2,1))];
            end
            new_points(end+1, 1) = p2(1);
            new_points(end, 2) = p2(2);
            p1 = prev;
            %traccio delle rette perpendicolari alle rette sulle quali giacciono i punti di raccordo
            %il loro punto di intersezione sarà il centro della circonferenza
            %il cui arco farà parte della traiettoria
            m1 = -(1/((points(i+1,2)-points(i,2))/(points(i+1,1)-points(i,1))));
            q1 = p1(2)-m1*p1(1);
            m2 = -(1/((points(i+2,2)-points(i+1,2))/(points(i+2,1)-points(i+1,1))));
            q2 = p2(2)-m2*p2(1);
            %se i punti di raccordo non giacciono sulla stessa retta posso definire la "fase di blending" 
            %altrimenti il moto sarà ancora rettilineo (gestione caso particolare)
            if m1~=m2 && abs(m1-m2)>0.1
                %0.1 è una soglia sotto la quale considero i due punti
                %collegati da una retta, anche se effettivamente la retta
                %su cui giaceva il primo non coincide con la seconda
                %gestione casi particolari
                if(m1==0)
                    if(abs(m2)==Inf)
                        sol = solve(y==p1(2), x==p2(1), [x,y]);
                    else
                        sol = solve(y==p1(2), y==m2*x+q2, [x,y]);
                    end
                elseif(m2==0)
                    if(abs(m1)==Inf)
                        sol = solve(x==p1(1), y==p2(2), [x,y]);
                    else
                        sol = solve(y==m1*x+q1, y==p2(2), [x,y]);
                    end
                elseif(abs(m1)==Inf)
                    sol = solve(x==p1(1), y==m2*x+q2, [x,y]);
                elseif(abs(m2)==Inf)
                    sol = solve(y==m1*x+q1, x==p2(1), [x,y]);
                else
                    sol = solve(y==m1*x+q1, y==m2*x+q2, [x,y]);
                end
                c = [double(sol.x), double(sol.y)];
                r = pdist([c; p1],'euclidean');
                infos(end+1,1) = c(1);
                infos(end,2) = c(2);
                infos(end,3) = r;
                infos(end,4) = 'b'; %98
            else
                infos(end+1,1) = (p1(2)-p2(2))/(p1(1)-p2(1));
                infos(end,2) = p1(2)-m*p1(1);
                infos(end,4) = 'l';
            end
            prev = p2;
        end
        if mod(p,2)==0
            i = i+1;
        end
    end
end