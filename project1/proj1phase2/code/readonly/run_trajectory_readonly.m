% Used for HKUST ELEC 5660

function run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9)

% Sensor parameters
fnoise = 1;           % Standard deviation of gaussian noise for external disturbance (N)
ifov   = 90;          % Camera field of view  

% Initialize simulation
global params;
params = quadModel_readonly(); % Quad model
x0     = zeros(13,1);
yaw0   = -30*pi/180;
pitch0 = -30*pi/180;
roll0  = -30*pi/180;
x0(1)  = 0; %x
x0(2)  = 0; %y
x0(3)  = 0; %z
x0(4)  = 0; %xdot
x0(5)  = 0; %ydot
x0(6)  = 0; %zdot
Quat0  = R_to_quaternion(ypr_to_R([yaw0 pitch0 roll0])');
x0(7)  = Quat0(1);  %qw
x0(8)  = Quat0(2);  %qx
x0(9)  = Quat0(3);  %qy
x0(10) = Quat0(4);  %qz
x0(11) = 0;         %p
x0(12) = 0;         %q
x0(13) = 0;         %r
true_s = x0;        % true state
F      = params.mass*params.grav;
M      = [0;0;0];

% Time
tstep    = 0.002;  % Time step for solving equations of motion // FIXME: not 0.01
cstep    = 0.01;   % Period of calling student code
vstep    = 0.05;   % visualization interval
time     = 0;      % current time
vis_time = 0;      % Time of last visualization
time_tol = 25;     % Maximum time that the quadrotor is allowed to fly

% Visualization
vis_init   = false;

% h1
thtraj     = [];
thprop1    = [];
thprop2    = [];
thprop3    = [];
thprop4    = [];
tharm1     = [];
tharm2     = [];
thfov1     = [];
thfov2     = [];
thfov3     = [];
thfov4     = [];
ehtraj     = [];
ehprop1    = [];
ehprop2    = [];
ehprop3    = [];
ehprop4    = [];
eharm1     = [];
eharm2     = [];
ehmap      = [];
ehwindow   = [];

% h2 h3
thpitch    = [];
throll     = [];

% h4 h5 h6
thvx       = [];
thvy       = [];
thvz       = [];
ehvx       = [];
ehvy       = [];
ehvz       = [];

% h7 h8 h9
thpx       = [];
thpy       = [];
thpz       = [];
ehpx       = [];
ehpy       = [];
ehpz       = [];


% Start Simulation run_trajectory_readonly
disp('Start Simulation ...');
while (1)
    
    % External disturbance
    Fd = randn(3,1) * fnoise;
    
    % Run simulation for cstep
    timeint = time:tstep:time+cstep;
    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F, M, Fd), timeint', true_s);
    true_s = xsave(end,:)';
    time = time + cstep;
    
    des_s = trajectory_generator(time);
    [F,M] = controller(time, true_s, des_s);
    
    if time >= time_tol
        break;
    end;    
    
    %% Rlot Results
    if time - vis_time > vstep
        
        %% Plot quad, fov, and estimated quad, estimated_map, and sliding window
        subplot(h1);
        hold on;
        if ~vis_init 
            grid on;    
            axis equal;        
            axis ([-10 10 -10 10 -1 4]);
        end
        %plot3(des_s(1),des_s(2),des_s(3),'m-');
        ll = 0.175;
        rr = 0.1;  
        ff = 0.3;
        nprop = 40;
        propangs = linspace(0,2*pi,nprop);        
        tR = QuatToRot(true_s(7:10))';
        tpoint1 = tR*[ll;0;0];
        tpoint2 = tR*[0;ll;0];
        tpoint3 = tR*[-ll;0;0];
        tpoint4 = tR*[0;-ll;0];
        tproppts = rr*tR*[cos(propangs);sin(propangs);zeros(1,nprop)];        
        twp1 = true_s(1:3) + tpoint1;
        twp2 = true_s(1:3) + tpoint2;
        twp3 = true_s(1:3) + tpoint3;
        twp4 = true_s(1:3) + tpoint4;
        tprop1 = tproppts + twp1*ones(1,nprop);
        tprop2 = tproppts + twp2*ones(1,nprop);
        tprop3 = tproppts + twp3*ones(1,nprop);
        tprop4 = tproppts + twp4*ones(1,nprop);    
        tfov0 = true_s(1:3);        
        tfov1 = tR * [ff;  ff * tan(ifov*pi/180/2);  ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov2 = tR * [ff;  ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2);  ff * tan(ifov*pi/180/2)] + true_s(1:3);
        eR = QuatToRot(des_s(7:10))';
        epoint1 = eR*[ll;0;0];
        epoint2 = eR*[0;ll;0];
        epoint3 = eR*[-ll;0;0];
        epoint4 = eR*[0;-ll;0];
        eproppts = rr*eR*[cos(propangs);sin(propangs);zeros(1,nprop)];        
        ewp1 = des_s(1:3) + epoint1;
        ewp2 = des_s(1:3) + epoint2;
        ewp3 = des_s(1:3) + epoint3;
        ewp4 = des_s(1:3) + epoint4;
        eprop1  = eproppts + ewp1*ones(1,nprop);
        eprop2  = eproppts + ewp2*ones(1,nprop);
        eprop3  = eproppts + ewp3*ones(1,nprop);
        eprop4  = eproppts + ewp4*ones(1,nprop);     
        emap    = [0;0;0];
        ewindow = [0;0;0];
        if ~vis_init
            thtraj = plot3(true_s(1), true_s(2), true_s(3), 'b-','LineWidth',3);                        
            tharm1 = line([twp1(1),twp3(1)],[twp1(2),twp3(2)],[twp1(3),twp3(3)],'Color','b');
            tharm2 = line([twp2(1),twp4(1)],[twp2(2),twp4(2)],[twp2(3),twp4(3)],'Color','b');
            thprop1 = plot3(tprop1(1,:),tprop1(2,:),tprop1(3,:),'r-');
            thprop2 = plot3(tprop2(1,:),tprop2(2,:),tprop2(3,:),'b-');
            thprop3 = plot3(tprop3(1,:),tprop3(2,:),tprop3(3,:),'b-');
            thprop4 = plot3(tprop4(1,:),tprop4(2,:),tprop4(3,:),'b-');
            thfov1  = line([tfov0(1) tfov1(1) tfov2(1)], [tfov0(2) tfov1(2) tfov2(2)], [tfov0(3) tfov1(3) tfov2(3)],'Color','k');
            thfov2  = line([tfov0(1) tfov2(1) tfov3(1)], [tfov0(2) tfov2(2) tfov3(2)], [tfov0(3) tfov2(3) tfov3(3)],'Color','k');
            thfov3  = line([tfov0(1) tfov3(1) tfov4(1)], [tfov0(2) tfov3(2) tfov4(2)], [tfov0(3) tfov3(3) tfov4(3)],'Color','k');
            thfov4  = line([tfov0(1) tfov4(1) tfov1(1)], [tfov0(2) tfov4(2) tfov1(2)], [tfov0(3) tfov4(3) tfov1(3)],'Color','k');
            ehtraj = plot3(des_s(1), des_s(2), des_s(3), 'g-','LineWidth',3);                                    
            eharm1 = line([ewp1(1),ewp3(1)],[ewp1(2),ewp3(2)],[ewp1(3),ewp3(3)],'Color','g');
            eharm2 = line([ewp2(1),ewp4(1)],[ewp2(2),ewp4(2)],[ewp2(3),ewp4(3)],'Color','g');
            ehprop1  = plot3(eprop1(1,:),eprop1(2,:),eprop1(3,:),'m-');
            ehprop2  = plot3(eprop2(1,:),eprop2(2,:),eprop2(3,:),'g-');
            ehprop3  = plot3(eprop3(1,:),eprop3(2,:),eprop3(3,:),'g-');
            ehprop4  = plot3(eprop4(1,:),eprop4(2,:),eprop4(3,:),'g-');        
            ehmap    = plot3(emap(1,:), emap(2,:), emap(3,:),'ko','MarkerSize',10,'MarkerFaceColor','k');
            ehwindow = plot3(ewindow(1,:), ewindow(2,:), ewindow(3,:),'ro','MarkerSize',5,'MarkerFaceColor','r');            
        else
            set(thtraj, 'XData', [get(thtraj, 'XData') true_s(1)]);             
            set(thtraj, 'YData', [get(thtraj, 'YData') true_s(2)]);                        
            set(thtraj, 'ZData', [get(thtraj, 'ZData') true_s(3)]);                                    
            set(thprop1,'XData',tprop1(1,:));
            set(thprop1,'YData',tprop1(2,:));
            set(thprop1,'ZData',tprop1(3,:));
            set(thprop2,'XData',tprop2(1,:));
            set(thprop2,'YData',tprop2(2,:));
            set(thprop2,'ZData',tprop2(3,:));
            set(thprop3,'XData',tprop3(1,:));
            set(thprop3,'YData',tprop3(2,:));
            set(thprop3,'ZData',tprop3(3,:));
            set(thprop4,'XData',tprop4(1,:));
            set(thprop4,'YData',tprop4(2,:));
            set(thprop4,'ZData',tprop4(3,:));
            set(tharm1,'XData',[twp1(1),twp3(1)]);
            set(tharm1,'YData',[twp1(2),twp3(2)]);
            set(tharm1,'ZData',[twp1(3),twp3(3)]);
            set(tharm2,'XData',[twp2(1),twp4(1)]);
            set(tharm2,'YData',[twp2(2),twp4(2)]);
            set(tharm2,'ZData',[twp2(3),twp4(3)]);   
            set(thfov1,'XData',[tfov0(1) tfov1(1) tfov2(1)]);
            set(thfov1,'YData',[tfov0(2) tfov1(2) tfov2(2)]);
            set(thfov1,'ZData',[tfov0(3) tfov1(3) tfov2(3)]);       
            set(thfov2,'XData',[tfov0(1) tfov2(1) tfov3(1)]);
            set(thfov2,'YData',[tfov0(2) tfov2(2) tfov3(2)]);
            set(thfov2,'ZData',[tfov0(3) tfov2(3) tfov3(3)]);   
            set(thfov3,'XData',[tfov0(1) tfov3(1) tfov4(1)]);
            set(thfov3,'YData',[tfov0(2) tfov3(2) tfov4(2)]);
            set(thfov3,'ZData',[tfov0(3) tfov3(3) tfov4(3)]);   
            set(thfov4,'XData',[tfov0(1) tfov4(1) tfov1(1)]);
            set(thfov4,'YData',[tfov0(2) tfov4(2) tfov1(2)]);
            set(thfov4,'ZData',[tfov0(3) tfov4(3) tfov1(3)]);               
            set(ehtraj, 'XData', [get(ehtraj, 'XData') des_s(1)]);
            set(ehtraj, 'YData', [get(ehtraj, 'YData') des_s(2)]);
            set(ehtraj, 'ZData', [get(ehtraj, 'ZData') des_s(3)]);
            set(ehprop1,'XData',eprop1(1,:));
            set(ehprop1,'YData',eprop1(2,:));
            set(ehprop1,'ZData',eprop1(3,:));
            set(ehprop2,'XData',eprop2(1,:));
            set(ehprop2,'YData',eprop2(2,:));
            set(ehprop2,'ZData',eprop2(3,:));
            set(ehprop3,'XData',eprop3(1,:));
            set(ehprop3,'YData',eprop3(2,:));
            set(ehprop3,'ZData',eprop3(3,:));
            set(ehprop4,'XData',eprop4(1,:));
            set(ehprop4,'YData',eprop4(2,:));
            set(ehprop4,'ZData',eprop4(3,:));
            set(eharm1,'XData',[ewp1(1),ewp3(1)]);
            set(eharm1,'YData',[ewp1(2),ewp3(2)]);
            set(eharm1,'ZData',[ewp1(3),ewp3(3)]);
            set(eharm2,'XData',[ewp2(1),ewp4(1)]);
            set(eharm2,'YData',[ewp2(2),ewp4(2)]);
            set(eharm2,'ZData',[ewp2(3),ewp4(3)]);
            set(ehmap,'XData',emap(1,:));
            set(ehmap,'YData',emap(2,:));
            set(ehmap,'ZData',emap(3,:));      
            set(ehwindow,'XData',ewindow(1,:));
            set(ehwindow,'YData',ewindow(2,:));
            set(ehwindow,'ZData',ewindow(3,:));                       
        end
        hold off;
        
        %% Plot roll oriengation
        subplot(h2);          
        true_ypr = R_to_ypr(quaternion_to_R(true_s(7:10))')*180/pi;      
        if ~vis_init 
            hold on;            
            throll  = plot(time, true_ypr(3),'r-','LineWidth',1);
            hold off;
            xlabel('Time (s)');
            ylabel('roll degree');
            axis ([0, time_tol, -45, 45]);               
        else
            hold on;
            set(throll, 'XData', [get(throll, 'XData') time]);
            set(throll, 'YData', [get(throll, 'YData') true_ypr(3)]);
            hold off;
        end          
        
        %% Plot pitch orientation
        subplot(h3);               
        if ~vis_init 
            hold on;                      
            thpitch = plot(time, true_ypr(2),'r-','LineWidth',1);         
            hold off;
            xlabel('Time (s)');
            ylabel('pitch degree');
            axis ([0, time_tol, -45, 45]);               
        else
            hold on;           
            set(thpitch, 'XData', [get(thpitch, 'XData') time]);
            set(thpitch, 'YData', [get(thpitch, 'YData') true_ypr(2)]);
            hold off;
        end            
       
        %% Plot body frame velocity
        subplot(h4);            
        true_v = true_s(4:6);
        des_v  = des_s(4:6);
        if ~vis_init 
            hold on;            
            thvx = plot(time,true_v(1),'r-','LineWidth',1);
            ehvx = plot(time,des_v(1),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('X World Velocity (m/s)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thvx, 'XData', [get(thvx, 'XData') time]);
            set(thvx, 'YData', [get(thvx, 'YData') true_v(1)]);
            set(ehvx, 'XData', [get(ehvx, 'XData') time]);                                
            set(ehvx, 'YData', [get(ehvx, 'YData') des_v(1)]);                          
            hold off;  
        end;
        
        subplot(h5);    
        if ~vis_init 
            hold on;            
            thvy = plot(time,true_v(2),'r-','LineWidth',1);
            ehvy = plot(time,des_v(2),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('Y World Velocity (m/s)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thvy, 'XData', [get(thvy, 'XData') time]);
            set(thvy, 'YData', [get(thvy, 'YData') true_v(2)]);          
            set(ehvy, 'XData', [get(ehvy, 'XData') time]);                          
            set(ehvy, 'YData', [get(ehvy, 'YData') des_v(2)]);  
            hold off;  
        end;
        subplot(h6);    
        if ~vis_init 
            hold on;            
            thvz = plot(time,true_v(3),'r-','LineWidth',1);
            ehvz = plot(time,des_v(3),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('Z World Velocity (m/s)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thvz, 'XData', [get(thvz, 'XData') time]);
            set(thvz, 'YData', [get(thvz, 'YData') true_v(3)]);
            set(ehvz, 'XData', [get(ehvz, 'XData') time]);                           
            set(ehvz, 'YData', [get(ehvz, 'YData') des_v(3)]);            
            hold off;  
        end;          
        
        %% Plot world frame position
        subplot(h7);            
        true_p = true_s(1:3);
        des_p  = des_s(1:3);
        if ~vis_init 
            hold on;            
            thpx = plot(time,true_p(1),'r-','LineWidth',1);
            ehpx = plot(time,des_p(1),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('X World Position (m)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thpx, 'XData', [get(thpx, 'XData') time]);
            set(thpx, 'YData', [get(thpx, 'YData') true_p(1)]);
            set(ehpx, 'XData', [get(ehpx, 'XData') time]);                                
            set(ehpx, 'YData', [get(ehpx, 'YData') des_p(1)]);                          
            hold off;  
        end;
        
        subplot(h8);    
        if ~vis_init 
            hold on;            
            thpy = plot(time,true_p(2),'r-','LineWidth',1);
            ehpy = plot(time,des_p(2),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('Y World Position (m)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thpy, 'XData', [get(thpy, 'XData') time]);
            set(thpy, 'YData', [get(thpy, 'YData') true_p(2)]);          
            set(ehpy, 'XData', [get(ehpy, 'XData') time]);                          
            set(ehpy, 'YData', [get(ehpy, 'YData') des_p(2)]);  
            hold off;  
        end;
        subplot(h9);    
        if ~vis_init 
            hold on;            
            thpz = plot(time,true_v(3),'r-','LineWidth',1);
            ehpz = plot(time,des_v(3),'b-','LineWidth',1);
            hold off;
            xlabel('Time (s) [Red: True; Blue: Des]');
            ylabel('Z World Position (m)');
            axis ([0, time_tol, -3, 3]);               
        else        
            hold on;
            set(thpz, 'XData', [get(thpz, 'XData') time]);
            set(thpz, 'YData', [get(thpz, 'YData') true_p(3)])
            set(ehpz, 'XData', [get(ehpz, 'XData') time]);
            set(ehpz, 'YData', [get(ehpz, 'YData') des_p(3)]);
            hold off;  
        end;       
        
        
        %% Render
        drawnow;        
        vis_time = time;             
        vis_init = true;
        
    end;
%     pause

end;
