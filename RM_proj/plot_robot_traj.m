function[h]=plot_robot_traj(Xtraj,k,robot_type,axis_vec,h)
if nargin<5 || isempty(h)
    h=figure();
end
if nargin<5
    axis_vec=[];
end

if robot_type==1
    cc='b';
else 
    cc='--r';
end

if k>0
    x = Xtraj(:,1:k);
    hold on;
    plot (x(1,:),x(2,:),cc,'LineWidth',2);
    plot_robot(x(:,end),robot_type,h);
else
    for kk=1:size(Xtraj,2)
        cla;
        plot_robot_traj(Xtraj,kk,robot_type,axis_vec,h);
        pause(0.05);
    end
end

if isempty(axis_vec)==0
    axis(axis_vec);
end
end