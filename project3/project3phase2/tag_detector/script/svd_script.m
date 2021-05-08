clc; clear;
load("2D_3D_points_data.mat")

pts_num = size(pts_3,1);

A = zeros(2*pts_num, 9);

for i = 1 : pts_num
    pt_2 = un_pts_2(i,:);
    pt_3 = pts_3(i,:);
    A(1+2*(i-1),:)=[pt_3(1), pt_3(2), 1, 0, 0, 0, -pt_3(1)*pt_2(1), -pt_3(2)*pt_2(1), -pt_2(1)];
    A(2+2*(i-1),:)=[0, 0, 0, pt_3(1), pt_3(2), 1, -pt_3(1)*pt_2(2), -pt_3(2)*pt_2(2), -pt_2(2)];
end
[U,S,V] = svd(A);
x = V(:,end)
H_hat = reshape(x, [3,3])

% H_ortho = [H_hat(:,1), H_hat(:,2), cross(H_hat(:,1), H_hat(:,2))];
% [U,S,V] = svd(H_ortho);
% R = U*V';
% t = H_hat(:,3)/norm(H_hat(:,3));

T_tilde = K\H_hat
R_tilde = [T_tilde(:,1),T_tilde(:,2),cross(T_tilde(:,1),T_tilde(:,2))]
[U,S,V] = svd(R_tilde);
R = U*V';
t = T_tilde(:,3)/norm(T_tilde(:,1));

% verify the result
% pts_3(:,3) = ones(pts_num,1);
T = [R(:,1), R(:,2),t];
H = T;
pts_2_estimated = zeros(pts_num,2);
for i = 1 : pts_num
    pt_3 = pts_3(i,:)';
    pt_2 = R*pt_3+t;
    pts_2_estimated(i,:)=[pt_2(1),pt_2(2)]/pt_2(3);
end