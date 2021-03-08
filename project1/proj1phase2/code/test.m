%{
This is a test script to check the efficiency of polyninomal trajectory
path: given waypoints
v: given velocity
T: total time
%}
path = [0, 0;
        1,0];
v = [0,0.1;
     -0.1,0];
T = 2;
 
% p(t) = a_0 + a_1 * t + a_2 * t^2 + a_3 * t^3
 
A = [1 0 0 0; ...
     1 T T^2 T^3;...
     0 1 0 0;...
     0 1 2*T 3*T^2];
 
p_x = A\[path(:,1); v(:,1)];
p_y = A\[path(:,2); v(:,2)];
 
traj = [];
for t = 0: 0.005 : T
    x = [1 t t^2 t^3]*p_x;
    y = [1 t t^2 t^3]*p_y;
    traj=[traj;[x,y]];
end
plot(traj(:,1), traj(:,2))