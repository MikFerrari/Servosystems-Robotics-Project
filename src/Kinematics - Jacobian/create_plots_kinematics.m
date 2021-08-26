% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name','Motion from home to first point of the shape - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,S(1,:), tt,S(2,:), tt,S(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper position [m]')
grid on
legend('x','y','z')

nexttile
plot(tt,Sp(1,:), tt,Sp(2,:), tt,Sp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper velocity [m/s]')
grid on
legend('x','y','z')

nexttile
plot(tt,Spp(1,:), tt,Spp(2,:), tt,Spp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('gripper acceleration [m/s^2]')
grid on
legend('x','y','z')

title(t,'Gripper coordinates from the home position to the beginning of the shape');


% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name','Motion from home to first point of the shape - Joint space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint position')
grid on
h1 = legend('$q_1$ \space $[rad]$','$q_2$ \space $[m]$','$q_3$ \space $[rad]$');
set(h1, 'Interpreter', 'latex')

nexttile
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint velocity')
grid on
h2 = legend('$\dot{q_1}$ \space $[rad/s]$','$\dot{q_2}$ \space $[m/s]$','$\dot{q_3}$ \space $[rad/s]$');
set(h2, 'Interpreter', 'latex')

nexttile
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint acceleration')
grid on
h3 = legend('$\ddot{q_1}$ \space $[rad/s^2]$','$\ddot{q_2}$ \space $[m/s^{2}]$','$\ddot{q_3}$ \space $[rad/s^2]$');
set(h3, 'Interpreter', 'latex')

title(t,'Joint coordinates from the home position to the beginning of the shape');


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name','Motion from home to first point of the shape - Curvilinear abscissa','NumberTitle','off')
plot(tt,pos, tt,vel, tt,acc, [tt(1) tt(end)],[0 0],'k')
title('Motion along the trajectory')
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('[cm] - [cm/s] - [cm/s^2]')
grid on
legend('displacement','velocity','acceleration','Location','northwest')

title('Curvilinear absissa, velocity and acceleration along the trajectory');


% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion from home to first point of the shape - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile

plot(tt,Sp(1,:), tt,xp_diff, tt,Spp(1,:), tt,xpp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('x direction')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')
   
nexttile
plot(tt,Sp(2,:), tt,yp_diff, tt,Spp(2,:), tt,ypp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('y direction')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')
   
nexttile
plot(tt,Sp(3,:), tt,zp_diff, tt,Spp(3,:), tt,zpp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('z direction')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')

title(t,{'Gripper coordinates from the home position to the beginning of the shape', ...
         'Analytical vs numerical solution'});


% PLOT JOINT COORDINATES - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion from home to first point of the shape - Joint space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt,Qp(1,:), tt,Qp_diff(1,:), tt,Qpp(1,:), tt,Qpp_diff(1,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 1')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')
   
nexttile
grid on
plot(tt,Qp(2,:), tt,Qp_diff(2,:), tt,Qpp(2,:), tt,Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 2')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')
   
nexttile
grid on
plot(tt,Qp(3,:), tt,Qp_diff(3,:), tt,Qpp(3,:), tt,Qpp_diff(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 3')
grid on
legend('velocity - analytical','velocity - numerical', ...
       'acceleration - analytical','acceleration - numerical','Location','southeast')

title(t,{'Joint coordinates from the home position to the beginning of the shape', ...
         'Analytical vs numerical solution'});