%{
This script is used to linearization the system dyanamics for 
EKF on quadrotor.
X_dot = f(x,u,n) with x in R^15, n in R^12
%}
syms x [15 1]
syms f [15 1]
syms w [3 1]
syms a [3 1]
syms n [12 1]
syms g 
G = [cos(x5) 0 -cos(x4)*sin(x5);
     0       1 sin(x4);
     sin(x5) 0 cos(x4)*cos(x5)];
Ginv = [[                           cos(x5)/(cos(x5)^2 + sin(x5)^2), 0,                            sin(x5)/(cos(x5)^2 + sin(x5)^2)]
[ (sin(x4)*sin(x5))/(cos(x4)*cos(x5)^2 + cos(x4)*sin(x5)^2), 1, -(cos(x5)*sin(x4))/(cos(x4)*cos(x5)^2 + cos(x4)*sin(x5)^2)]
[          -sin(x5)/(cos(x4)*cos(x5)^2 + cos(x4)*sin(x5)^2), 0,            cos(x5)/(cos(x4)*cos(x5)^2 + cos(x4)*sin(x5)^2)]];

Rot = [cos(x6)*cos(x5)-sin(x4)*sin(x6)*sin(x5), -cos(x4)*sin(x6), cos(x6)*sin(x5)+cos(x5)*sin(x4)*sin(x6);
       cos(x5)*sin(x6)+cos(x6)*sin(x4)*sin(x5), cos(x4)*cos(x6), sin(x6)*sin(x5)-cos(x6)*cos(x5)*sin(x4);
       -cos(x4)*sin(x5), sin(x4), cos(x4)*cos(x5)];
f(1:3) = x(7:9);
f(4:6) = Ginv*(w - x(10:12) - n(1:3));
f(7:9) = [0,0,g]' + Rot*(a - x(13:15) - n(4:6));
f(10:12) = n(7:9);
f(13:15) = n(10:12);

fdx = jacobian(f,x)
fdn = jacobian(f,n)
% fdx = jacobian(f(2),x)