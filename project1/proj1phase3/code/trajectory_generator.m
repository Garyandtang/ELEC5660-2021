function s_des = trajectory_generator(t, path, h, map)
persistent p_all
persistent des_positions
persistent time_point
persistent time_interval
if nargin > 1 % pre-process can be done here (given waypoints). Pre-define the entire trajectory.

% visualize the 2D grid map
subplot(h);
% start point
plot3(map(1, 1)-0.5, map(1, 2)-0.5, map(1, 3)-0.5, 'k.');
hold on;
% obstacles
for obs_cnt = 2: size(map, 1) - 1
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.2 map(obs_cnt, 2)-0.8], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.8 map(obs_cnt, 2)-0.2], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
    hold on;
    ox = map(obs_cnt ,1) - 0.9;
    oy = map(obs_cnt ,2) - 0.9;
    oz = map(obs_cnt ,3) - 0.9;
    plotcube([0.8,0.8,0.8], [ox,oy,oz],1,[0.7,0.7,0.7]);
    grid minor
    set(gca,'xtick',[-100:1:100])
    set(gca,'ytick',[-100:1:100])
    grid off;
    grid on;       
    axis equal;        
    axis ([-1 6 -1 10 0 4]);
    hold on;
end
% target point
plot3(map(obs_cnt+1, 1)-0.5, map(obs_cnt+1, 2)-0.5, map(obs_cnt+1, 3)-0.5, 'r*');
hold on;
%% time assignment
debug_flag = 0;
total_time = 25;
waypoint_diff = path(2:end,:)-path(1:end-1,:);
path_length = sqrt(sum(waypoint_diff.^2,2));
cum_length = cumsum(path_length);
time_point = total_time * cum_length/cum_length(end);
time_point = [0;time_point]; % size: length(path) * 1
time_interval = time_point(2:end)-time_point(1:end-1); % size: time_point-1
% time_point = [0:2.5:2.5*(size(path,1)-1)]';
% time_interval = time_point(2:end)-time_point(1:end-1); % size: time_point-1
%% cost function
N = 8; % minimum snap
M = length(path)-1;
Q_all = zeros(N*M,N*M); %block diagonal matrix
p_all = zeros(N*M,size(path,2));
for k = 1:M
    for i = 4:8
        for j = 4:8
            Q_all(N*(k-1)+i,N*(k-1)+j) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7)*time_interval(k)^(i+j-7);
        end
    end
end
%% Constraints
% model waypoints constraints f_j^{(0)}(T_j)=x_j^{(0)}
d_positions = kron(path, ones(2,1));
d_positions = d_positions(2:end-1,:);
A_positions = zeros(size(d_positions,1),M*N);

for j = 1 : M
    A_positions((1+2*(j-1)):(2+2*(j-1)),(8*j-7):(8*j))= ...
        [1, 0, 0, 0, 0, 0, 0, 0; ...
        1, time_interval(j)^1,time_interval(j)^2, time_interval(j)^3,time_interval(j)^4,time_interval(j)^5,time_interval(j)^6,time_interval(j)^7];
end

% continuity cosntraints
% position + velocity + acceleration + ya
KK = 4; % hyper parameter on no. of constraints
A_continuity = zeros(KK*(M-1),M*N);
d_continuity = zeros(KK*(M-1), size(path,2));
for j = 1 : M-1
    A_continuity((1+KK*(j-1)):(KK+KK*(j-1)),(1+8*(j-1)):(16+8*(j-1)))= ...
        [1, time_interval(j)^1,time_interval(j)^2, time_interval(j)^3,time_interval(j)^4,time_interval(j)^5,time_interval(j)^6,time_interval(j)^7,-1,0,0,0,0,0,0,0; ...
        0, time_interval(j)^0,2*time_interval(j)^1, 3*time_interval(j)^2,4*time_interval(j)^3,5*time_interval(j)^4,6*time_interval(j)^5,7*time_interval(j)^6,0,-1,0,0,0,0,0,0; ...
        0, 0*time_interval(j)^0,2*time_interval(j)^0, 6*time_interval(j)^1,12*time_interval(j)^2,20*time_interval(j)^3,30*time_interval(j)^4,42*time_interval(j)^5,0,0,-2,0,0,0,0,0;...
        0, 0*time_interval(j)^0,0*time_interval(j)^0, 6*time_interval(j)^0,24*time_interval(j)^1,60*time_interval(j)^2,120*time_interval(j)^3,210*time_interval(j)^4,0,0,0,-6,0,0,0,0;];
end
A_eq = [A_positions;A_continuity];
d_eq = [d_positions;d_continuity];
f = zeros(M*N,1);
p_all(:,1) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,1));
p_all(:,2) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,2));
p_all(:,3) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,3));
%% for debuging
if debug_flag == 1
    figure (4)
    traj = [];
    for t = 0:0.1:time_point(end)
        for i = 1 : size(time_point,1)
            if t >= time_point(i) && t <time_point(i+1)
                time = time_point(i);
                index = i;
                T = t - time;
                p = p_all((1+8*(index-1)):(8+8*(index-1)),:);
                point = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
                traj = [traj;point];
            end
        end
    end
    plot3(traj(:,1),traj(:,2),traj(:,3))
    hold on
    scatter3(path(:,1),path(:,2),path(:,3))
    return
end
else % output desired trajectory here (given time)
%% output desired state
% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate
s_des = zeros(13,1);
if t > time_point(end)
    s_des(1:3) = des_positions;
    return
end
for i = 1 : size(time_point,1)
    if t >= time_point(i) && t <time_point(i+1)
        time = time_point(i);
        index = i;
        break
    end
end
T = t - time;
p = p_all((1+8*(index-1)):(8+8*(index-1)),:);
s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
des_positions = s_des(1:3);
s_des(4:6) = [0,1*T^0,2*T^1,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*p;
s_des(7:9) = [0,0*T^0,2*T^0,6*T^1,12*T^2,20*T^3,30*T^4,42*T^5]*p;    
end

end


