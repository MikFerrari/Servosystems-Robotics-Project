%% Plot kinematics results

% RENAME VARIABLES TO PLOT MORE EASILY
% Gripper position
x_grip = reshape(SS(1,end,:),1,[]);
y_grip = reshape(SS(2,end,:),1,[]);
z_grip = reshape(SS(3,end,:),1,[]);
% Gripper velocity
xp_grip = reshape(SSp(1,end,:),1,[]);   xp_diff_grip = reshape(SSp_diff(1,end,:),1,[]);
yp_grip = reshape(SSp(2,end,:),1,[]);   yp_diff_grip = reshape(SSp_diff(2,end,:),1,[]);
zp_grip = reshape(SSp(3,end,:),1,[]);   zp_diff_grip = reshape(SSp_diff(3,end,:),1,[]);
% Gripper acceleration
xpp_grip = reshape(SSpp(1,end,:),1,[]);   xpp_diff_grip = reshape(SSpp_diff(1,end,:),1,[]);
ypp_grip = reshape(SSpp(2,end,:),1,[]);   ypp_diff_grip = reshape(SSpp_diff(2,end,:),1,[]);
zpp_grip = reshape(SSpp(3,end,:),1,[]);   zpp_diff_grip = reshape(SSpp_diff(3,end,:),1,[]);
% Gripper motion along the trajectory
pos_grip = reshape(PP(1,end,:),1,[]);
% vel_grip = reshape(VV(1,end,:),1,[]);   
% acc_grip = reshape(AA(1,end,:),1,[]);   
vel_diff_grip = reshape(VV_diff(1,end,:),1,[]);
acc_diff_grip = reshape(AA_diff(1,end,:),1,[]);


% INITIAL & FINAL CONFIGURATION + TRAJECTORY
figure('name','3D representation xyz + trajectory','NumberTitle','off')

if task == 1    
    p1 = DH_plot_robot(Q_home,D,A,alpha,gcf,'r');           % P1
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_initial_shape,D,A,alpha,gcf,'b');  % P2
    
elseif task == 2
    p1 = DH_plot_robot(Q_initial_shape,D,A,alpha,gcf,'b');  % P2
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3

elseif task == 3
    p1 = DH_plot_robot(Q_final_shape,D,A,alpha,gcf,'k');    % P3
    view(34,38)
    hold on
    p2 = DH_plot_robot(Q_home,D,A,alpha,gcf,'r');           % P1
end

for j = 1:nLinks
    hold on
    p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
end

if task == 1    
    legend([p1(1) p2(1) p3],'P1 pose','P2 pose','joint trajectory');
elseif task == 2
    legend([p1(1) p2(1) p3],'P2 pose','P3 pose','joint trajectory');
elseif task == 3
    legend([p1(1) p2(1) p3],'P3 pose','P1 pose','joint trajectory');
end

grid on
title('RRR 3D manipulator poses and joint trajectory');
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(27,26) % Point of view: (azimut,elevation)

%%
% GRIPPER POSITION, VELOCITY, ACCELERATION - ONLY ANALYTICAL SOLUTION
figure('Name','Motion along x','NumberTitle','off')
plot(tt,x_grip,  tt,xp_grip, tt,xpp_grip, [tt(1) tt(end)],[0 0],'k')
set(gca, 'XLimSpec', 'Tight')
grid on
title('Motion along x-axis')
legend('displacement','velocity','acceleration','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along y','NumberTitle','off')
plot(tt,y_grip,  tt,yp_grip, tt,ypp_grip, [tt(1) tt(end)],[0 0],'k')
set(gca, 'XLimSpec', 'Tight')
grid on
title('Motion along y-axis')
legend('displacement','velocity','acceleration','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along z','NumberTitle','off')
plot(tt,z_grip,  tt,zp_grip, tt,zpp_grip, [tt(1) tt(end)],[0 0],'k')
set(gca, 'XLimSpec', 'Tight')
grid on
title('Motion along z-axis')
legend('displacement','velocity','acceleration','Location','northeast')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along the trajectory','NumberTitle','off')
% plot(tt,pos_grip,  tt,vel_grip, tt,acc_grip, [tt(1) tt(end)],[0 0],'k')
plot(tt,pos_grip, [tt(1) tt(end)],[0 0],'k')
set(gca, 'XLimSpec', 'Tight')
grid on
title('Motion along the trajectory')
legend('displacement','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')


% % JOINT POSITION, VELOCITY, ACCELERATION IN JOINT SPACE - ONLY ANALYTICAL SOLUTION
% figure('Name','Motion in the joint space','NumberTitle','off')
% plot(tt,Q(1,:), tt,Q(2,:))
% hold on
% plot(tt,Qp(1,:), tt,Qp(2,:), 'Linewidth',1.5)
% plot(tt,Qpp(1,:), tt,Qpp(2,:), [tt(1) tt(end)],[0 0],'k')
% title('Motion in the joint space')
% legend('$\alpha$','$\beta$','$\stackrel{.}{\alpha}$','$\stackrel{.}{\beta}$', ...
%        '$\stackrel{..}{\alpha}$','$\stackrel{..}{\beta}$', ...
%        'interpreter','latex','Location','southeast')
% xlabel('time'), ylabel('degrees')


% GRIPPER POSITION, VELOCITY, ACCELERATION - ANALYTICAL VS NUMERICAL SOLUTION
figure('Name','Motion along x','NumberTitle','off')
plot(tt,xp_grip, tt(1:end-1),xp_diff_grip, tt,xpp_grip, tt(1:end-1),xpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end-1)])
grid on
title('Motion along x-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along y','NumberTitle','off')
plot(tt,yp_grip, tt(1:end-1),yp_diff_grip, tt,ypp_grip, tt(1:end-1),ypp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end-1)])
grid on
title('Motion along y-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along z','NumberTitle','off')
plot(tt,zp_grip, tt(1:end-1),zp_diff_grip, tt,zpp_grip, tt(1:end-1),zpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
set(gca, 'XLimSpec', 'Tight')
grid on
title('Motion along z-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northeast')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along the trajectory','NumberTitle','off')
plot(tt(1:end-1),vel_diff_grip, tt(1:end-2),acc_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end-2)])
% set(gca, 'XLimSpec', 'Tight');
grid on
title('Motion along the trajectory')
legend('velocity','acceleration','Location','southwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')


% % JOINT POSITION, VELOCITY, ACCELERATION IN JOINT SPACE - ANALYTICAL VS NUMERICAL SOLUTION
% figure('Name','Motion in the joint space','NumberTitle','off')
% plot(tt,Qp(1,:), tt(1:end-1),Qp_diff(1,:), tt,Qp(2,:), tt(1:end-1),Qp_diff(2,:),'Linewidth',1.5)
% hold on
% plot(tt,Qpp(1,:), tt(1:end-1),Qpp_diff(1,:), tt,Qpp(2,:), tt(1:end-1),Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
% title('Motion in the joint space')
% legend('$\stackrel{.}{\alpha}$','$\stackrel{.}{\alpha}$ - diff', ...
%        '$\stackrel{.}{\beta}$','$\stackrel{.}{\beta}$ - diff', ...
%        '$\stackrel{..}{\alpha}$','$\stackrel{..}{\alpha}$ - diff', ...
%        '$\stackrel{..}{\beta}$','$\stackrel{..}{\beta}$ - diff', ...
%        'interpreter','latex','Location','southeast')
% xlabel('time'), ylabel('degrees')



%% Plot dynamics results

%{
% FORCES/TORQUES APPLIED BY THE ACTUATORS
figure('Name',strcat("Actuator torques [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Actuator torques - [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
actuatorTorques = reshape(phi_actuators_array_0,4,1,nPoints,[]);
titles = {'','Torque joint 1','Torque joint 2','Torque joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorTorques(i,:), [tt(1) tt(end)],[0 0],'k')
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% POWER DEVELOPED BY THE ACTUATORS
figure('Name',strcat("Actuator power [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Actuator power [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
actuatorPower = reshape(Wq,4,1,nPoints,[]);
titles = {'','Power joint 1','Power joint 2','Power joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorPower(i,:), [tt(1) tt(end)],[0 0],'k')
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% KINETIC, POTENTIAL & TOTAL ENERGY
figure('Name',strcat("Kinetic and potential energy - [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Kinetic and potential energy - [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
titles = {'Potential Energy','Kinetic Energy','Total Energy (E_k+E_p)'};
EE = [Ep_tot; Ek_tot; E_tot];
for i = 1:3
    nexttile
    plot(tt, EE(i,:))
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
figure('Name','Power equality','NumberTitle','off')
plot(tt(1:end-1),E_tot_diff, tt,W_tot, [tt(1) tt(end)],[0 0],'k')
grid on
title('Derivative of total energy vs power of actuators and external forces')
legend('dE_{tot}/dt','W_{q}+W_{ext}','Location','southwest')
xlabel('time [s]'), ylabel('power [W]')

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name','Power equality','NumberTitle','off')
plot(tt(1:end-1),Ek_tot_diff, tt,W_tot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
title('Derivative of kinetic energy vs power of actuators, external and weight forces')
legend('dE_{k}/dt','W_{q}+W_{ext}+W_{weight}','Location','southwest')
xlabel('time [s]'), ylabel('power [W]')
%}