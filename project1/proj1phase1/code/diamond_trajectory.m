function s_des = diamond_trajectory(t, true_s)
    
    s_des = zeros(11,1);
    
    t1=t; Period=12; %Diamond Period
    Seg=sqrt(2); Vel=Seg*4/Period;
    x_des=4/25*t; x_vdes=4/25; x_ades=0;
    T1=Period/4; T2=Period/2; T3=3*Period/4;
    
    while(t1>Period)
        t1=t1-Period;
    end

    if(t1<=T1)
        y_ades=0;
        z_ades=0;
        y_vdes=Vel;
        z_vdes=Vel;
        y_des=y_vdes*t1;
        z_des=z_vdes*t1;        
    elseif(T1<t1 && t1<=T2)
        y_ades=0;
        z_ades=0;
        y_vdes=-1*Vel;
        z_vdes=Vel;    
        y_des=Seg+y_vdes*(t1-Period/4);
        z_des=Seg+z_vdes*(t1-Period/4);
    elseif(T2<t1 && t1<=T3)
        y_ades=0;
        z_ades=0;
        y_vdes=-1*Vel;
        z_vdes=-1*Vel;
        y_des=y_vdes*(t1-Period/2);
        z_des=2*Seg+z_vdes*(t1-Period/2);
    else
        y_ades=0;
        z_ades=0;
        y_vdes=Vel;
        z_vdes=-1*Vel;
        y_des=-Seg+y_vdes*(t1-3*Period/4);
        z_des=Seg+z_vdes*(t1-3*Period/4);
    end

    yaw_des = mod(0.2 * pi * t,2 * pi);
    dyaw_des = 0.2 * pi;
    
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
