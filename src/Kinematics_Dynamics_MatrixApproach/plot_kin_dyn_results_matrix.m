%% Plot kinematics results

% RENAME VARIABLES TO PLOT MORE EASILY
% Gripper position
x_grip = reshape(SS(1,end,:),1,[]);
y_grip = reshape(SS(2,end,:),1,[]);
z_grip = reshape(SS(3,end,:),1,[]);
% Gripper velocity

xp_grip = reshape(SSp(1,end,:),1,[]);   xp_diff_grip = [0 reshape(SSp_diff(1,end,:),1,[])];
yp_grip = reshape(SSp(2,end,:),1,[]);   yp_diff_grip = [0 reshape(SSp_diff(2,end,:),1,[])];
zp_grip = reshape(SSp(3,end,:),1,[]);   zp_diff_grip = [0 reshape(SSp_diff(3,end,:),1,[])];

% Gripper acceleration
xpp_grip = reshape(SSpp(1,end,:),1,[]);   xpp_diff_grip = [0 reshape(SSpp_diff(1,end,:),1,[])];
ypp_grip = reshape(SSpp(2,end,:),1,[]);   ypp_diff_grip = [0 reshape(SSpp_diff(2,end,:),1,[])];
zpp_grip = reshape(SSpp(3,end,:),1,[]);   zpp_diff_grip = [0 reshape(SSpp_diff(3,end,:),1,[])];

% Gripper motion along the trajectory
pos_grip = reshape(PP(1,end,:),1,[]);  
vel_diff_grip = reshape(VV_diff(1,end,:),1,[]);
acc_diff_grip = reshape(AA_diff(1,end,:),1,[]);

% INITIAL & FINAL CONFIGURATION + TRAJECTORY
figure('name','3D representation xyz + trajectory','NumberTitle','off')

if task == 1    
    p1 = DH_plot_robot(Q_home,D,A,alpha,gcf,'r');           % P1
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_initial_shape,D,A,alpha,gcf,'b');  % P2
    for j = 1:nLinks
        hold on
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
    end
    legend([p1(1) p2(1) p3],'P1 pose','P2 pose','joint trajectory');
    
elseif task == 2
    p1 = DH_plot_robot(Q_initial_shape,D,A,alpha,gcf,'b');  % P2
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3
    for j = 1:nLinks
        hold on
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
    end
    legend([p1(1) p2(1) p3],'P2 pose','P3 pose','joint trajectory');

elseif task == 3
    p1 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_home,D,A,alpha,gcf,'r');           % P1
    for j = 1:nLinks
        hold on
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
    end
    legend([p1(1) p2(1) p3],'P3 pose','P1 pose','joint trajectory');
end

axis equal
grid on
title('Manipulator poses and joint trajectory');
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(30,20) % Point of view: (azimut,elevation)
if task == 1
    task_title = " from P1 to P2 ";
elseif task == 2
    task_title = " from P2 to P3 ";
else 
    task_title = " from P3 to P1 ";
end
    
% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name',strcat("Motion",task_title,"- Gripper space"),'NumberTitle','off')
t = tiledlayout(3,1);
fsz=12;

nexttile
plot(tt,x_grip,  tt,y_grip, tt,z_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on

nexttile
plot(tt,xp_grip,  tt,yp_grip, tt,zp_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity [m/s]')
grid on

nexttile
plot(tt,xpp_grip,  tt,ypp_grip, tt,zpp_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration [m/s^2]')
grid on

leg = legend('$x$ \space $axis$','$y$ \space $axis$','$z$ \space $axis$','Orientation', 'Horizontal');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
leg.Layout.Tile = 'south';

title(t,strcat('Gripper coordinates',task_title));
% exportgraphics(gcf, 'myfigure_ex.pdf')



% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name',strcat('Motion',task_title,'- Joint space'),'NumberTitle','off')
t = tiledlayout(3,1);
fsz=10;

nexttile
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position')
grid on
h1 = legend('$q_1$ \space $[rad]$','$q_2$ \space $[m]$','$q_3$ \space $[rad]$','Location','northeastoutside');
set(h1, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity')
grid on
h2 = legend('$\dot{q_1}$ \space $[rad/s]$','$\dot{q_2}$ \space $[m/s]$','$\dot{q_3}$ \space $[rad/s]$','Location','northeastoutside');
set(h2, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration')
grid on
h3 = legend('$\ddot{q_1}$ \space $[rad/s^2]$','$\ddot{q_2}$ \space $[m/s^{2}]$','$\ddot{q_3}$ \space $[rad/s^2]$','Location','northeastoutside');
set(h3, 'Interpreter', 'latex','FontSize',fsz)

title(t,strcat('Joint coordinates',task_title));
% set(gcf,'Visible','on')


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name',strcat('Motion along the trajectory',task_title,'- Curvilinear abscissa'),'NumberTitle','off')
plot(tt,pos_grip, tt(1:end-1),vel_diff_grip, tt(1:end-2),acc_diff_grip, [tt(1) tt(end)],[0 0],'k')
title('Motion along the trajectory')
xlim([tt(1) tt(end)])
xlabel('time [s]')
grid on
leg=legend('$displacement$ \space $[m]$','$velocity$ \space $[m/s]$','$acceleration$ \space $[m/s^2]$','Location','northwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)

title(strcat('Displacement, velocity and acceleration along the trajectory',task_title));

% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name',strcat('Motion',task_title,' - Gripper space'),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,xp_grip, tt,xp_diff_grip, tt,xpp_grip, tt,xpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('x direction')
grid on
% legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical','Location','northeastoutside')
h00 = legend('$\dot{x}$ $-$ $ analytical$ \space $[m/s]$','$\dot{x}$ $-$ $ numerical$ \space $[m/s]$',...
           '$\ddot{x}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{x}$ $-$ $ numerical$ \space $[m/s^2]$',...
           'Location','northeastoutside');
set(h00, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,yp_grip, tt,yp_diff_grip, tt,ypp_grip, tt,ypp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('y direction')
grid on
h01 = legend('$\dot{y}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{y}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{y}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{y}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h01, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,zp_grip, tt,zp_diff_grip, tt,zpp_grip, tt,zpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('z direction')
grid on
h02 = legend('$\dot{z}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{z}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{z}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{z}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h02, 'Interpreter', 'latex','FontSize',fsz)

title(t,{'Gripper coordinates', ...
         'Analytical vs numerical solution'});

% PLOT JOINT COORDINATES - ANALYTICAL VS NUMERICAL SOLUTION
figure('name',strcat('Motion',task_title,'- Joint space'),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Qp(1,:), tt,Qp_diff(1,:), tt,Qpp(1,:), tt,Qpp_diff(1,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 1')
grid on
h4 = legend('$\dot{q_1}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_1}$ $-$ $ numerical$ \space $[rad/s]$',...
           '$\ddot{q_1}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_1}$ $-$ $ numerical$ \space $[rad/s^2]$',...
           'Orientation','Vertical','Location','northeastoutside');
set(h4, 'Interpreter', 'latex','FontSize',fsz)
% legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

nexttile
grid on
plot(tt,Qp(2,:), tt,Qp_diff(2,:), tt,Qpp(2,:), tt,Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 2')
grid on
h5 = legend('$\dot{q_2}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_2}$ $-$ $ numerical$ \space $[rad/s]$',...
           '$\ddot{q_2}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_2}$ $-$ $ numerical$ \space $[rad/s^2]$',...
           'Location','northeastoutside');
set(h5, 'Interpreter', 'latex','FontSize',fsz)

nexttile
grid on
plot(tt,Qp(3,:), tt,Qp_diff(3,:), tt,Qpp(3,:), tt,Qpp_diff(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 3')
grid on
h6 =legend('$\dot{q_3}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_3}$ $-$ $ numerical$ \space $[rad/s]$',...
          '$\ddot{q_3}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_3}$ $-$ $ numerical$ \space $[rad/s^2]$',...
          'Location','northeastoutside');
set(h6, 'Interpreter', 'latex','FontSize',fsz)

title(t,{strcat('Joint coordinates',task_title), ...
         'Analytical vs numerical solution'});   

%% Plot dynamics results

%%{
% FORCES/TORQUES APPLIED BY THE ACTUATORS
figure('Name',strcat('Actuator torques and forces',task_title),'NumberTitle','off')
t = tiledlayout(3,1);
% t = tiledlayout(3,1);

title(t,strcat('Actuator torques and forces',task_title));
actuatorTorques = reshape(phi_actuators_array_0,4,1,nPoints,[]);
titles = {'','Joint 1','Joint 2','Joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorTorques(i,:))
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
    xlim([tt(1) tt(end)])
end
% set(gcf,'Visible','on')

% POWER DEVELOPED BY THE ACTUATORS
figure('Name',strcat('Actuator power',task_title),'NumberTitle','off')
t = tiledlayout(3,1);
title(t,strcat('Actuator power',task_title));
actuatorPower = reshape(Wq,4,1,nPoints,[]);
titles = {'','Joint 1','Joint 2','Joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorPower(i,:))
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
    xlim([tt(1) tt(end)])
end

% KINETIC, POTENTIAL & TOTAL ENERGY
figure('Name',strcat('Kinetic and potential energy',task_title),'NumberTitle','off')
t = tiledlayout(3,1);
title(t,strcat('Kinetic and potential energy',task_title));
titles = {'Potential Energy','Kinetic Energy','Total Energy (E_k+E_p)'};
EE = [Ep_tot; Ek_tot; E_tot];
for i = 1:3
    nexttile
    plot(tt, EE(i,:))
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
    xlim([tt(1) tt(end)])
end

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
fsz=11;
figure('Name',strcat('Power equality 1',task_title),'NumberTitle','off')
plot(tt(1:end),[0 E_tot_diff], tt,W_tot, [tt(1) tt(end)],[0 0],'k')
grid on
title(strcat('Conservation of mechanical energy',task_title))
% legend('dE_{tot}/dt','W_{q}+W_{ext}','Location','southwest')

leg = legend('${dE_k}/{dt}$','$W_{q} + W_{ext}$','Location','southwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
xlabel('time [s]'), ylabel('power [W]')
xlim([tt(1) tt(end)])

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name',strcat('Power equality 2',task_title),'NumberTitle','off')
plot(tt(1:end),[0 Ek_tot_diff], tt,W_tot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
title(strcat('Conservation of mechanical energy',task_title))
leg = legend('${dE_k}/{dt}$','$W_{q} + W_{ext} + W_{weight}$','Location','southwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
xlabel('time [s]'), ylabel('power [W]')
xlim([tt(1) tt(end)])
%}
