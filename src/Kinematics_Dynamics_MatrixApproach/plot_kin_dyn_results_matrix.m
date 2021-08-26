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

% xp_diff_grip = diff(S(1,:),[],2)/dT;
% yp_diff_grip = diff(S(1,:),[],2)/dT;
% zp_diff_grip = diff(S(1,:),[],2)/dT;

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

% for j = 1:nLinks
%     hold on
%     p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
% end

% if task == 1    
%     legend([p1(1) p2(1) p3],'P1 pose','P2 pose','joint trajectory');
% elseif task == 2
%     legend([p1(1) p2(1) p3],'P2 pose','P3 pose','joint trajectory');
% elseif task == 3
%     legend([p1(1) p2(1) p3],'P3 pose','P1 pose','joint trajectory');
% end

grid on
title('RRR 3D manipulator poses and joint trajectory');
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(27,26) % Point of view: (azimut,elevation)

%%
%%{
% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name','Position, velocity, acceleration - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,x_grip,  tt,y_grip, tt,z_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper position [m]')
grid on
legend('x','y','z')

nexttile
plot(tt,xp_grip,  tt,yp_grip, tt,zp_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper velocity [m/s]')
grid on
legend('x','y','z')

nexttile
plot(tt,xpp_grip,  tt,ypp_grip, tt,zpp_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper acceleration [m/s^2]')
grid on
legend('x','y','z')

title(t,'Gripper coordinates');


% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name','Motion - Joint space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint position')
grid on
legend('x [rad]','y [m]','z [rad]')

nexttile
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint velocity')
grid on
legend('x [rad/s]','y [m/s]','z [rad/s]')

nexttile
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint acceleration')
grid on
legend('x [rad/s^2]','y [m/s^2]','z [rad/s^2]')

title(t,'Joint coordinates');


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name','Motion along the trajectory - Curvilinear abscissa','NumberTitle','off')
plot(tt,pos_grip, tt(1:end-1),vel_diff_grip, tt(1:end-2),acc_diff_grip, [tt(1) tt(end)],[0 0],'k')
title('Motion along the trajectory')
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('[cm] - [cm/s] - [cm/s^2]')
grid on
legend('displacement','velocity','acceleration','Location','northwest')

title('Curvilinear absissa, velocity and acceleration along the trajectory');


% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,xp_grip, tt,xp_diff_grip, tt,xpp_grip, tt,xpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('x direction')
grid on
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

nexttile
plot(tt,yp_grip, tt,yp_diff_grip, tt,ypp_grip, tt,ypp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('y direction')
grid on
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

nexttile
plot(tt,zp_grip, tt,zp_diff_grip, tt,zpp_grip, tt,zpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('z direction')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')

title(t,{'Gripper coordinates', ...
         'Analytical vs numerical solution'});

% PLOT JOINT COORDINATES - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion - Joint space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Qp(1,:), tt,Qp_diff(1,:), tt,Qpp(1,:), tt,Qpp_diff(1,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 1')
grid on
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

nexttile
grid on
plot(tt,Qp(2,:), tt,Qp_diff(2,:), tt,Qpp(2,:), tt,Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 2')
grid on
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

nexttile
grid on
plot(tt,Qp(3,:), tt,Qp_diff(3,:), tt,Qpp(3,:), tt,Qpp_diff(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 3')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')

title(t,{'Joint coordinates', ...
         'Analytical vs numerical solution'});   

%% Plot dynamics results

%%{
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
%%