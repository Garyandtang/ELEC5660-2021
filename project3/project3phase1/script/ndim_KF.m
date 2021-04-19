close all;
clear all;
clc;
A = 0.99; 
C = 2; 
Q = 1;
R = 2; 
N = 2;

% check result with DARE
[X,L,G] = dare(A',C',Q,R);
optimal_Kk = inv(A)*G';
optimal_P = X - X*C'*inv(C*X*C' +R)*C*X;


x = zeros(1, N);
xp = zeros(1,N); % prediction of x
xp_co = zeros(1,N); % prediction covariance
xe = zeros(1,N); % estimate of x
xe_co = zeros(1,N); % estimate covariance

a = 0.01;
% initialization
x(1) = 0;
xp(1) = 0;
xp_co(1) = X;
xe_co(1) = 5;
k_list = [];

K_ = 0;

x_store = [];
x_hat_store = [];
for j = 1:500
for k=1:N-1
    
    % system dynamics
    
    x(k+1) = A*x(k) + sqrt(Q)*randn;
    yk = C*x(k) + sqrt(R)*randn;
    
%     % update k with gradient descent
%     K_pre = 5;
%   
%     K_ = 0;
%     while (abs(K_ - K_pre) > 10^(-7))
%         delta_K = -(2*(C*xp_co(k)*C+R)*K_ - xp_co(k)*C*2);
%         K_pre = K_;
%         K_ = K_ + a*delta_K;
%         k_list = [k_list; K_];
%         
%     end
%     Kk = K_;
    % KF measurement update
    
    Kk = xp_co(k)*C*inv(C*xp_co(k)*C+R);
    xe(k) = xp(k) + Kk*[yk - C*xp(k)];
    xe_co(k) = xp_co(k) -Kk*C*xp_co(k);
    
    % KF time update
    
    xp(k+1) = A*xe(k);
    xp_co(k+1) = A*xe_co(k)*A + Q;
    K_ = Kk;
end
x_store = [x_store; x(k)];
x_hat_store = [x_hat_store, xe(k)];
end
x_var = var(x_store)
x_hat_var = var(x_hat_store)
xe_co(1)


figure(1)
subplot(2,1,1)
plot(1:N-1, x(1:N-1), '--',1:N-1, xe(1:N-1));
legend('xk', 'estimate of xk');
hold on;
subplot(2,1,2)
error = x-xe;
plot(1:N-1,error(1:N-1),'--',1:N-1, xe_co(1:N-1));
legend('estimation error', 'error covariance')