function s_des = circle_trajectory(t, true_s)
    
    s_des = zeros(11,1);

    omega=25;      
    x_des=4*cos(t*omega/180*pi);
    y_des=4*sin(t*omega/180*pi);
    z_des=3/25*t;          

    x_vdes=-omega/180*pi*4*sin(t*omega/180*pi);   
    y_vdes= omega/180*pi*4*cos(t*omega/180*pi);
    z_vdes=3/25;
    
    x_ades=-omega/180*pi*omega/180*pi*4*cos(t*omega/180*pi);   
    y_ades=-omega/180*pi*omega/180*pi*4*sin(t*omega/180*pi);
    z_ades=0;
    
    %desired yaw angle in the flight
    yaw_des = mod(0.1 * pi * t,2 * pi);
    dyaw_des = 0.1 * pi;
    
    s_des(1)=x_des;
    s_des(2)=y_des;
    s_des(3)=z_des;
    s_des(4)=x_vdes;
    s_des(5)=y_vdes;
    s_des(6)=z_vdes;
    s_des(7)=x_ades;
    s_des(8)=y_ades;
    s_des(9)=z_ades;
    s_des(10)=yaw_des;
    s_des(11)=dyaw_des;
end
