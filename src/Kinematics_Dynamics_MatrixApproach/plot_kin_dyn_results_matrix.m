%% Plot kinematics results
close all
plot_kin_directory = 'C:\Users\monti\OneDrive - unibs.it\Servosystems & Robotics Project\src\Kinematics_Dynamics_MatrixApproach\plot kin dyn results\kinematics';
plot_dyn_directory = 'C:\Users\monti\OneDrive - unibs.it\Servosystems & Robotics Project\src\Kinematics_Dynamics_MatrixApproach\plot kin dyn results\dynamics';

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
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'Color','#32a838');
    end
    legend([p1(1) p2(1) p3],'P1 pose','P2 pose','joint trajectory', 'Location','southoutside','Orientation', 'Horizontal');
    view(30,20) % Point of view: (azimut,elevation)
    
elseif task == 2
    p1 = DH_plot_robot(Q_initial_shape,D,A,alpha,gcf,'b');  % P2
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3
    for j = 1:nLinks
        hold on
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'Color','#32a838');
    end
    legend([p1(1) p2(1) p3],'P2 pose','P3 pose','joint trajectory', 'Location','southoutside','Orientation', 'Horizontal');
    view(33,42) % Point of view: (azimut,elevation)

elseif task == 3
    p1 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_home,D,A,alpha,gcf,'r');           % P1
    for j = 1:nLinks
        hold on
        p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'Color','#32a838');
    end
    legend([p1(1) p2(1) p3],'P3 pose','P1 pose','joint trajectory', 'Location','southoutside','Orientation', 'Horizontal');
    view(30,20) % Point of view: (azimut,elevation)
end

axis equal
grid on
title('Manipulator poses and joint trajectory');
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
if task == 1
    task_title = " from P1 to P2 ";
elseif task == 2
    task_title = " from P2 to P3 ";
else 
    task_title = " from P3 to P1 ";
end
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_robot_traj.pdf'))

    
% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name',strcat("Motion",task_title,"- Gripper space"),'NumberTitle','off')
t = tiledlayout(3,1);
fsz=12;

nexttile
plot(tt,x_grip,  tt,y_grip, tt,z_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on

nexttile
plot(tt,xp_grip, tt,yp_grip, tt,zp_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity [m/s]')
grid on

nexttile
plot(tt,xpp_grip, tt,ypp_grip, tt,zpp_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration [m/s^2]')
grid on

leg = legend('$x$ \space $axis$','$y$ \space $axis$','$z$ \space $axis$','Orientation', 'Horizontal');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
leg.Layout.Tile = 'south';

newcolors = {'#7E2F8E','#ff8c00','#32a838'};
colororder(newcolors)
title(t,strcat('Gripper coordinates',task_title));
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_PosVelAcc_gripper.pdf'))

% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name',strcat('Motion',task_title,'- Joint space'),'NumberTitle','off')
t = tiledlayout(3,1);
fsz=10;

nexttile
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:))
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position')
grid on
h1 = legend('$q_1$ \space $[rad]$','$q_2$ \space $[m]$','$q_3$ \space $[rad]$','Location','northeastoutside');
set(h1, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:))
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity')
grid on
h2 = legend('$\dot{q_1}$ \space $[rad/s]$','$\dot{q_2}$ \space $[m/s]$','$\dot{q_3}$ \space $[rad/s]$','Location','northeastoutside');
set(h2, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:))
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration')
grid on
h3 = legend('$\ddot{q_1}$ \space $[rad/s^2]$','$\ddot{q_2}$ \space $[m/s^{2}]$','$\ddot{q_3}$ \space $[rad/s^2]$','Location','northeastoutside');
set(h3, 'Interpreter', 'latex','FontSize',fsz)

lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
colororder(newcolors)
title(t,strcat('Gripper coordinates',task_title));
title(t,strcat('Joint coordinates',task_title));
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_PosVelAcc_joints.pdf'))


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name',strcat('Motion along the trajectory',task_title,'- Curvilinear abscissa'),'NumberTitle','off')
plot(tt,pos_grip, tt(1:end-1),vel_diff_grip, tt(1:end-2),acc_diff_grip, [tt(1) tt(end)],[0 0],'k')
title('Motion along the trajectory')
xlim([tt(1) tt(end)])
xlabel('time [s]')
grid on
leg=legend('$displacement$ \space $[m]$','$velocity$ \space $[m/s]$','$acceleration$ \space $[m/s^2]$','Location','northwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
colororder(newcolors)
title(t,strcat('Gripper coordinates',task_title));
title(strcat('Displacement, velocity and acceleration along the trajectory',task_title));
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_curv_abscissa.pdf'))

% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name',strcat('Motion',task_title,' - Gripper space'),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,xp_grip, tt,xp_diff_grip, tt,xpp_grip, tt,xpp_diff_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('x direction')
grid on
% legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical','Location','northeastoutside')
h00 = legend('$\dot{x}$ $-$ $ analytical$ \space $[m/s]$','$\dot{x}$ $-$ $ numerical$ \space $[m/s]$',...
           '$\ddot{x}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{x}$ $-$ $ numerical$ \space $[m/s^2]$',...
           'Location','northeastoutside');
set(h00, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,yp_grip, tt,yp_diff_grip, tt,ypp_grip, tt,ypp_diff_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('y direction')
grid on
h01 = legend('$\dot{y}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{y}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{y}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{y}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h01, 'Interpreter', 'latex','FontSize',fsz)

nexttile
plot(tt,zp_grip, tt,zp_diff_grip, tt,zpp_grip, tt,zpp_diff_grip)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('z direction')
grid on
h02 = legend('$\dot{z}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{z}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{z}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{z}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h02, 'Interpreter', 'latex','FontSize',fsz)
lines = findobj(gcf,'Type','Line');

title(t,strcat('Gripper coordinates',task_title));
title(t,{'Gripper coordinates', ...
         'Analytical vs numerical solution'});
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_kin_gripper_debug.pdf'))


% PLOT JOINT COORDINATES - ANALYTICAL VS NUMERICAL SOLUTION
figure('name',strcat('Motion',task_title,'- Joint space'),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Qp(1,:), tt,Qp_diff(1,:), tt,Qpp(1,:), tt,Qpp_diff(1,:))
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
plot(tt,Qp(2,:), tt,Qp_diff(2,:), tt,Qpp(2,:), tt,Qpp_diff(2,:))
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 2')
grid on
h5 = legend('$\dot{q_2}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_2}$ $-$ $ numerical$ \space $[rad/s]$',...
           '$\ddot{q_2}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_2}$ $-$ $ numerical$ \space $[rad/s^2]$',...
           'Location','northeastoutside');
set(h5, 'Interpreter', 'latex','FontSize',fsz)

nexttile
grid on
plot(tt,Qp(3,:), tt,Qp_diff(3,:), tt,Qpp(3,:), tt,Qpp_diff(3,:))
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 3')
grid on
h6 =legend('$\dot{q_3}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_3}$ $-$ $ numerical$ \space $[rad/s]$',...
          '$\ddot{q_3}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_3}$ $-$ $ numerical$ \space $[rad/s^2]$',...
          'Location','northeastoutside');
set(h6, 'Interpreter', 'latex','FontSize',fsz)

title(t,{strcat('Joint coordinates',task_title), ...
         'Analytical vs numerical solution'});   
exportgraphics(gcf, strcat(plot_kin_directory,'\task',num2str(task),'_kin_joints_debug.pdf'))


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
    xlabel('time'), 
    if i == 2 || i == 4
        ylabel('Nm')
    else 
        ylabel('N')
    end
    xlim([tt(1) tt(end)])
end
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
exportgraphics(gcf, strcat(plot_dyn_directory,'\task',num2str(task),'_ActuatorActions.pdf'))


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
    xlabel('time'), ylabel('W')
    xlim([tt(1) tt(end)])
end
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
exportgraphics(gcf, strcat(plot_dyn_directory,'\task',num2str(task),'_ActuatorPower.pdf'))


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
    xlabel('time'), ylabel('J')
    xlim([tt(1) tt(end)])
end
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
exportgraphics(gcf, strcat(plot_dyn_directory,'\task',num2str(task),'_KinPotMecEnergy.pdf'))


% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
fsz=11;
figure('Name',strcat('Power equality 1',task_title),'NumberTitle','off')
plot(tt(1:end),[0 E_tot_diff], tt,W_tot, [tt(1) tt(end)],[0 0],'k')
grid on
title(strcat('Conservation of mechanical energy',task_title))
leg = legend('${dE_{tot}}/{dt}$','$W_{q} + W_{ext}$','Location','southwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
xlabel('time [s]'), ylabel('power [W]')
xlim([tt(1) tt(end)])
exportgraphics(gcf, strcat(plot_dyn_directory,'\task',num2str(task),'_EnergyConservation1.pdf'))


% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name',strcat('Power equality 2',task_title),'NumberTitle','off')
plot(tt(1:end),[0 Ek_tot_diff], tt,W_tot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
title(strcat('Conservation of mechanical energy',task_title))
leg = legend('${dE_{k}}/{dt}$','$W_{q} + W_{ext} + W_{weight}$','Location','southwest');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
xlabel('time [s]'), ylabel('power [W]')
xlim([tt(1) tt(end)])
exportgraphics(gcf, strcat(plot_dyn_directory,'\task',num2str(task),'_EnergyConservation2.pdf'))

%}
