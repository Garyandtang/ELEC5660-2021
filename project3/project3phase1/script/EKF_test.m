%{
Author: Jiawei Tang
Matlab simulation of EKF for state estimation with tag detection and imu.
Data is recorded by rosbag.
This is a test script for ELEC5660 Project 3 Phase 1.
%}
clear;clc;
load("matlab.mat");

z_list = measurement_list;
% system parameters
Q = 0.1*eye(12,12);
% Q(7:12,7:12) = eye(6);
Rt = 0.001*eye(6,6);
Sigma = 1*eye(15);
x_est = zeros(15,1);

C = zeros(6,15);
C(:, 1:6) = eye(6);
% r = mvnrnd(zeros(6,1),Rt,1)';
K_list = [];
x_est_list = [];

step = 1;
dt = 0.0025;
for i = 1: 23
    % measurement update
    inno = z_list(i,:)' - C*x_est;
    inno(4) = EulerAngleClamp(inno(4));
    inno(5) = EulerAngleClamp(inno(5));
    inno(6) = EulerAngleClamp(inno(6));
    K = Sigma*C'*inv(C*Sigma*C'+Rt);
    x_est = x_est + K*inno;
    Sigma = Sigma - K*C*Sigma;
    K_list = [K_list;K];
    x_est_list = [x_est_list; x_est'];
    % time update
    count = 1;
    while count <= odom_dt(i)
        f = zeros(15,1);
        f(1:3) = x_est(7:9);
        omega = u_list(step,1:3)';
        acc = u_list(step, 4:6)';
        
        G = [cos(x_est(5)), 0, -cos(x_est(4))*sin(x_est(5));
             0,             1, sin(x_est(4));
             sin(x_est(5)), 0, cos(x_est(4))*cos(x_est(5))];
        Ginv = inv(G);
        f(4:6) = Ginv * (omega - x_est(10:12));
        x4 = x_est(4); x5 = x_est(5); x6 = x_est(6);
        Rot = [cos(x6)*cos(x5)-sin(x4)*sin(x6)*sin(x5), -cos(x4)*sin(x6), cos(x6)*sin(x5)+cos(x5)*sin(x4)*sin(x6);
               cos(x5)*sin(x6)+cos(x6)*sin(x4)*sin(x5), cos(x4)*cos(x6), sin(x6)*sin(x5)-cos(x6)*cos(x5)*sin(x4);
               -cos(x4)*sin(x5), sin(x4), cos(x4)*cos(x5)];
        f(7:9) = Rot * (acc - x_est(13:15)) + [0;0;9.8];
        A_t = zeros(15,15); 
        U_t = zeros(15,12); 
        G_inv_dot = [0, omega(3)*cos(x4)-omega(1)*sin(x5),0;
                     omega(1)*sin(x5)-omega(3)*cos(x5)-(omega(3)*cos(x5)*sin(x4)*sin(x4))/(cos(x4)*cos(x4))+(omega(1)*sin(x4)*sin(x4)*sin(x4))/(cos(x4)*cos(x4)), ...
                     (omega(1) * cos(x5) * sin(x4)) / cos(x4) + (omega(3) * sin(x4) * sin(x5)) / cos(x4), 0;
                     (omega(3)*cos(x5)*sin(x4))/(cos(x4)*cos(x4))-(omega(1)*sin(x4)*sin(x5))/(cos(x4)*cos(x4)),...
                     -(omega(1)*cos(x5))/cos(x4)-(omega(3)*sin(x5))/cos(x4),0];
                 
        R_dot = [acc(2)*sin(x4)*sin(x6)+acc(3)*cos(x4)*cos(x5)*sin(x6)-acc(1)*cos(x4)*sin(x5)*sin(x6),...
                 acc(3)*(cos(x5)*cos(x6)-sin(x4)*sin(x5)*sin(x6))-acc(1)*(cos(x6)*sin(x5)+cos(x5)*sin(x4)*sin(x6)),...
                  -acc(1)*(cos(x5)*sin(x6)+cos(x6)*sin(x4)*sin(x5))-acc(3)*(sin(x5)*sin(x6)-cos(x5)*cos(x6)*sin(x4))-acc(3)*cos(x4)*cos(x6);...
                  acc(1)*cos(x4)*cos(x6)*sin(x5)-acc(3)*cos(x4)*cos(x5)*cos(x6)-acc(2)*cos(x6)*sin(x4),...
                  acc(3)*(cos(x5)*sin(x6)+cos(x6)*sin(x4)*sin(x5))-acc(1)*(sin(x5)*sin(x6)-cos(x5)*cos(x6)*sin(x4)),...
                  acc(1)*(cos(x5)*cos(x6)-sin(x4)*sin(x5)*sin(x6))+acc(3)*(cos(x6)*sin(x5)+cos(x4)*sin(x4)*sin(x6))-acc(2)*cos(x4)*sin(x6);...
                  acc(2)*cos(x4)-acc(3)*cos(x5)*sin(x4)+acc(1)*sin(x4)*sin(x5),...
                  -acc(1)*cos(x4)*cos(x5)-acc(3)*cos(x4)*sin(x5), 0];
        A_t(1:3, 7:9) = eye(3);
        A_t(4:6, 4:6) = G_inv_dot;
        A_t(7:9, 4:6) = R_dot;
        A_t(4:6, 10:12) = -Ginv;
        A_t(7:9, 13:15) = - Rot;
        
        U_t(4:6,1:3) = -Ginv;
        U_t(7:9,4:6) = Rot;
        U_t(10:15,7:12) = eye(6);
        
        F_t = eye(15)+dt*A_t;
        V_t = dt*U_t;
        x_est = x_est + dt*f;
        x_est(4) = EulerAngleClamp(x_est(4));
        x_est(5) = EulerAngleClamp(x_est(5));
        x_est(6) = EulerAngleClamp(x_est(6));
        Sigma = F_t*Sigma*F_t' + V_t * Q * V_t';
        x_est = x_est + dt*f;
        x_est(4) = EulerAngleClamp(x_est(4));
        x_est(5) = EulerAngleClamp(x_est(5));
        x_est(6) = EulerAngleClamp(x_est(6));
        step = step + 1;
        count = count + 1;
    end
   
end


scatter3(z_list(:,1),z_list(:,2),z_list(:,3),'g');
hold on
scatter3(x_est_list(:,1), x_est_list(:,2), x_est_list(:,3),'b')
% scatter3(cpp_est(:,1), cpp_est(:,2), cpp_est(:,3))

function [result] =  EulerAngleClamp(angle)
result = atan2(sin(angle), cos(angle));
end