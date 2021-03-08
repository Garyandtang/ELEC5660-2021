theta = 0;
R = [cos(theta) -sin(theta) 0;
     sin(theta) cos(theta) 0;
     0          0          1];

R_trans = R'; % transpose R

R_dot_parmeter = [-sin(theta) -cos(theta) 0
                   cos(theta) -sin(theta) 0
                   0           0          0];

T_bc = [0 0 1 0.05;
        -1 0 0 0.1;
        0 -1 0 0.02;
        0 0 0 1];
    
R_z = [0 1 0;
       -1 0 0;
       0 0 1];
R_x = [ 1 0 0 ;
        0 0 1;
        0 -1 0];
    