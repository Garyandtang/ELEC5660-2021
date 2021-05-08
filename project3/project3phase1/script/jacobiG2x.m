syms x [21 1] real
syms z [6 1] real
Rot = [cos(x6)*cos(x5)-sin(x4)*sin(x6)*sin(x5), -cos(x4)*sin(x6), cos(x6)*sin(x5)+cos(x5)*sin(x4)*sin(x6);
       cos(x5)*sin(x6)+cos(x6)*sin(x4)*sin(x5), cos(x4)*cos(x6), sin(x6)*sin(x5)-cos(x6)*cos(x5)*sin(x4);
       -cos(x4)*sin(x5), sin(x4), cos(x4)*cos(x5)];
Rot_key = [cos(x21)*cos(x20)-sin(x19)*sin(x21)*sin(x20), -cos(x19)*sin(x21), cos(x21)*sin(x20)+cos(x20)*sin(x19)*sin(x21);
       cos(x20)*sin(x21)+cos(x21)*sin(x19)*sin(x20), cos(x19)*cos(x21), sin(x21)*sin(x20)-cos(x21)*cos(x20)*sin(x19);
       -cos(x19)*sin(x20), sin(x19), cos(x19)*cos(x20)];

R_tran = [[ cos(x20)*cos(x21) - sin(x19)*sin(x20)*sin(x21), cos(x20)*sin(x21) + cos(x21)*sin(x19)*sin(x20), -cos(x19)*sin(x20)]
[                             -cos(x19)*sin(x21),                              cos(x19)*cos(x21),           sin(x19)]
[ cos(x21)*sin(x20) + cos(x20)*sin(x19)*sin(x21), sin(x20)*sin(x21) - cos(x20)*cos(x21)*sin(x19),  cos(x19)*cos(x20)]];

R = [[ (cos(x5)*sin(x6) + cos(x6)*sin(x4)*sin(x5))*(cos(x20)*sin(x21) + cos(x21)*sin(x19)*sin(x20)) + (cos(x5)*cos(x6) - sin(x4)*sin(x5)*sin(x6))*(cos(x20)*cos(x21) - sin(x19)*sin(x20)*sin(x21)) + cos(x4)*cos(x19)*sin(x5)*sin(x20), cos(x4)*cos(x6)*(cos(x20)*sin(x21) + cos(x21)*sin(x19)*sin(x20)) - cos(x19)*sin(x4)*sin(x20) - cos(x4)*sin(x6)*(cos(x20)*cos(x21) - sin(x19)*sin(x20)*sin(x21)), (cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6))*(cos(x20)*cos(x21) - sin(x19)*sin(x20)*sin(x21)) + (sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4))*(cos(x20)*sin(x21) + cos(x21)*sin(x19)*sin(x20)) - cos(x4)*cos(x5)*cos(x19)*sin(x20)]
[                                                                        cos(x19)*cos(x21)*(cos(x5)*sin(x6) + cos(x6)*sin(x4)*sin(x5)) - cos(x4)*sin(x5)*sin(x19) - cos(x19)*sin(x21)*(cos(x5)*cos(x6) - sin(x4)*sin(x5)*sin(x6)),                                                                        sin(x4)*sin(x19) + cos(x4)*cos(x6)*cos(x19)*cos(x21) + cos(x4)*cos(x19)*sin(x6)*sin(x21),                                                                        cos(x4)*cos(x5)*sin(x19) + cos(x19)*cos(x21)*(sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4)) - cos(x19)*sin(x21)*(cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6))]
[ (cos(x5)*sin(x6) + cos(x6)*sin(x4)*sin(x5))*(sin(x20)*sin(x21) - cos(x20)*cos(x21)*sin(x19)) + (cos(x5)*cos(x6) - sin(x4)*sin(x5)*sin(x6))*(cos(x21)*sin(x20) + cos(x20)*sin(x19)*sin(x21)) - cos(x4)*cos(x19)*cos(x20)*sin(x5), cos(x19)*cos(x20)*sin(x4) + cos(x4)*cos(x6)*(sin(x20)*sin(x21) - cos(x20)*cos(x21)*sin(x19)) - cos(x4)*sin(x6)*(cos(x21)*sin(x20) + cos(x20)*sin(x19)*sin(x21)), (cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6))*(cos(x21)*sin(x20) + cos(x20)*sin(x19)*sin(x21)) + (sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4))*(sin(x20)*sin(x21) - cos(x20)*cos(x21)*sin(x19)) + cos(x4)*cos(x5)*cos(x19)*cos(x20)]
 ];

z(1:3) = R_tran*(x(1:3)-x(16:18));
z(4) = asin(R(2,3));
z(5) = atan2((-R(2,1)/cos(z(4))),(R(2,2)/cos(z(4))))
z(6) = atan2((-R(1,3)/cos(z(4))),(R(3,3)/cos(z(4))))
z = ccode(z)
% % z(5) = angle((sin(x4)*sin(x19) + cos(x4)*sin(x5)*sin(x19)*1i + cos(x4)*cos(x6)*cos(x19)*cos(x21) + cos(x5)*cos(x6)*cos(x19)*sin(x21)*1i - cos(x5)*cos(x19)*cos(x21)*sin(x6)*1i + cos(x4)*cos(x19)*sin(x6)*sin(x21) - cos(x6)*cos(x19)*cos(x21)*sin(x4)*sin(x5)*1i - cos(x19)*sin(x4)*sin(x5)*sin(x6)*sin(x21)*1i)/(1 - (cos(x4)*cos(x5)*sin(x19) + cos(x19)*cos(x21)*(sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4)) - cos(x19)*sin(x21)*(cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6)))^2)^(1/2))
% % z(6)= angle(-((cos(x20) + sin(x20)*1i)*(cos(x6)*cos(x21)*sin(x5) + sin(x5)*sin(x6)*sin(x21) + cos(x4)*cos(x5)*cos(x19)*1i - cos(x5)*cos(x6)*sin(x4)*sin(x21) + cos(x5)*cos(x21)*sin(x4)*sin(x6) + cos(x6)*sin(x5)*sin(x19)*sin(x21)*1i - cos(x21)*sin(x5)*sin(x6)*sin(x19)*1i + cos(x5)*cos(x6)*cos(x21)*sin(x4)*sin(x19)*1i + cos(x5)*sin(x4)*sin(x6)*sin(x19)*sin(x21)*1i)*1i)/(1 - (cos(x4)*cos(x5)*sin(x19) + cos(x19)*cos(x21)*(sin(x5)*sin(x6) - cos(x5)*cos(x6)*sin(x4)) - cos(x19)*sin(x21)*(cos(x6)*sin(x5) + cos(x5)*sin(x4)*sin(x6)))^2)^(1/2))
%  
% zdx = simplify(jacobian(z,x))
% Ct_1_1 = zdx(1:3,1:3)
% Ct_1_6 = zdx(1:3,16:18)
% Ct_1_7 = zdx(1:3,19:21)
% Ct_2_2 = zdx(4:6,4:6)
% Ct_2_7 = zdx(4:6,19:21)

% clear;
% 
% syms phi theta psi real
% syms phi2 theta2 psi2 real
% R1 = [cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta) -cos(phi)*sin(psi) cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
%      cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta) cos(phi)*cos(psi) sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
%      -cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)];
% 
%  
% R2 = [cos(psi2)*cos(theta2)-sin(phi2)*sin(psi2)*sin(theta2) -cos(phi2)*sin(psi2) cos(psi2)*sin(theta2)+cos(theta2)*sin(phi2)*sin(psi2);
%      cos(theta2)*sin(psi2)+cos(psi2)*sin(phi2)*sin(theta2) cos(phi2)*cos(psi2) sin(psi2)*sin(theta2)-cos(psi2)*cos(theta2)*sin(phi2);
%      -cos(phi2)*sin(theta2) sin(phi2) cos(phi2)*cos(theta2)];
% 
% R = simplify(R1'*R2);
% 
% X = simplify(asin(R(2,3)));
% Z = simplify(atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi)));
% Y = simplify(atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));
% 
% rzdx = simplify(jacobian([X;Y;Z], [phi theta psi phi2 theta2 psi2]))







