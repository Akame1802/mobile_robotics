function traj_vg = Visibility_Graph(nodes, figs_map, map_plot)

s = []; %nodi di partenza
t = []; %nodi di arrivo
weights = [];

%creo gli archi noti collengando tra loro i vertici delle figure
%Data una figura, posso formare archi fra coppie dei suoi vertici
%escludendo le coppie i cui archi attraversano la figura stessa
for k=2:size(figs_map(:))-1
    p0 = figs_map(k-1)+1;
    p1 = figs_map(k-1)+2;
    p2 = figs_map(k-1)+3;
    p3 = figs_map(k);
    %dati i vertici in quest'ordine, so che gli archi saranno p0-p1,
    %p0-p2,p1-p3,p2-p3 (e questi delimitano la figura considerata)
    s = [s,p0,p0,p1,p2];
    t = [t,p1,p2,p3,p3];
    weights = [weights, ...
        pdist([nodes(p0,1),nodes(p0,2);nodes(p1,1),nodes(p1,2)],'euclidean'),...
        pdist([nodes(p0,1),nodes(p0,2);nodes(p2,1),nodes(p2,2)],'euclidean'),...
        pdist([nodes(p3,1),nodes(p3,2);nodes(p1,1),nodes(p1,2)],'euclidean'),...
        pdist([nodes(p2,1),nodes(p2,2);nodes(p3,1),nodes(p3,2)],'euclidean')];
end

n = size(figs_map, 2);
for k=1:n
    if(k==1) %questo è il punto di start
        first = 1;
    else 
        %primo vertice della prima figura
        first = figs_map(k-1)+1;
    end
    for i=first:figs_map(k) %la i varia sui punti della figura k (punto o figura piana)
        for j=figs_map(k)+1:figs_map(end)
            %j varia tra il primo punto della figura successiva a k e l'ultimo punto dell'ultima figura
            %devono essere verificati tutti gli archi i-j
            linkable = true;
            l = pdist([nodes(i,1),nodes(i,2);nodes(j,1),nodes(j,2)],'euclidean');
            %occorre gestire i casi particolari per i punti di start e goal:
            %m1 è il coefficiente angolare del segmento ij, ovvero l'arco
            %di cui sto testando la realizzabilità; m2 è il coeff. angolare
            %del segmento che si forma una volta fra i e start (h=1), una volta
            %fra i e goal (h=n)
            for h=1:n
                if(h==1)
                    if(i~=1)
                        m1 = (nodes(i,2)-nodes(j,2))/(nodes(i,1)-nodes(j,1));
                        m2 = (nodes(i,2)-nodes(1,2))/(nodes(i,1)-nodes(1,1));
                        if(m1==m2) %se m1!=m2 so con certezza che il punto di start non si trova sul segmento
                            pm = [nodes(i,1)+nodes(j,1)/2, nodes(i,2)+nodes(j,2)/2];
                            d = pdist([pm(1),pm(2);nodes(1,1),nodes(1,2)],'euclidean');
                            if(d<l/2) %il punto appartiene al segmento
                                linkable = false;
                                break
                            end
                        end
                    end
                    continue
                end
                if(h==n)
                    if(i~=n)
                        m1 = (nodes(i,2)-nodes(j,2))/(nodes(i,1)-nodes(j,1));
                        m2 = (nodes(i,2)-nodes(end,2))/(nodes(i,1)-nodes(end,1));
                        if(m1==m2)
                            pm = [nodes(i,1)+nodes(j,1)/2, nodes(i,2)+nodes(j,2)/2];
                            d = pdist([pm(1),pm(2);nodes(end,1),nodes(end,2)],'euclidean');
                            if(d<l/2)
                                linkable = false;
                                break
                            end
                        end
                    end
                    continue
                end
                %gestione archi che partono dai vertici delle figure piane
                p0 = figs_map(h-1)+1;
                p1 = figs_map(h-1)+2;
                p2 = figs_map(h-1)+3;
                p3 = figs_map(h);
                %nel caso in cui i o j siano vertici della figura
                %che sto testando, non considero i lati della figura che
                %includono almeno uno dei due
                if(j~=p0 && j~=p1 && i~=p0 && i~=p1)
                    if(Intersects(nodes(i,:), nodes(j,:), nodes(p0,:), nodes(p1,:)))
                        linkable = false;
                        break;
                    end
                end
                if(j~=p0 && j~=p2 && i~=p0 && i~=p2)
                    if(Intersects(nodes(i,:), nodes(j,:), nodes(p0,:), nodes(p2,:)))
                        linkable = false;
                        break;
                    end
                end
                if(j~=p1 && j~=p3 && i~=p1 && i~=p3)
                    if(Intersects(nodes(i,:), nodes(j,:), nodes(p1,:), nodes(p3,:)))
                        linkable = false;
                        break;
                    end
                end
                if(j~=p2 && j~=p3 && i~=p2 && i~=p3)
                    if(Intersects(nodes(i,:), nodes(j,:), nodes(p2,:), nodes(p3,:)))
                        linkable = false;
                        break;
                    end
                end
            end
            if(linkable) 
                %creo l'arco che parte da i e arriva in j il cui peso è la
                %loro distanza euclidea
                s = [s, i];
                t = [t, j];
                weights = [weights, l];
            end
        end
    end
end

G = graph(s,t,weights);
start = 1; goal = size(nodes, 1);
traj_vg = shortestpath(G, start, goal);

%plots
map_plot; hold on;
for link=1:size(s(:))
    p1 = nodes(s(link),:);
    p2 = nodes(t(link),:);
    plot([p1(1),p2(1)], [p1(2),p2(2)], 'b--', 'LineWidth', 0.7);
end
for node=1:size(traj_vg, 2)-1
    p1 = nodes(traj_vg(node),:);
    p2 = nodes(traj_vg(node+1),:);
    plot([p1(1),p2(1)], [p1(2),p2(2)], 'g-', 'LineWidth', 1);
end

nexttile;
p = plot(G, 'EdgeLabel', G.Edges.Weight);
highlight(p, traj_vg, 'EdgeColor', 'g', 'LineWidth',2);
title('Visibility Graph');
set(map_plot,'visible','on');
waitforbuttonpress;

% la traiettoria così generata è una sequenza di spezzate
traj_points = [];
for i=1:size(traj_vg,2)
    traj_points = [traj_points; nodes(traj_vg(i),1), nodes(traj_vg(i),2)];
end
traj_vg = traj_points;

end