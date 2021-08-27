% INITIAL & FINAL CONFIGURATION + TRAJECTORY
figure('name','3D representation xyz + trajectory','NumberTitle','off')

if task == 1    
    p1 = plot3(S_home(1),S_home(2),S_home(3),'oc');                               % Home point
    hold on
    p2 = plot3(S_initial_shape(1),S_initial_shape(2),S_initial_shape(3),'om');    % First point of the shape
    p3 = plot3(S(1,:),S(2,:),S(3,:),'k');                                         % Trajectory
    
    plot_robot(num2cell(Q(:,1)),L,angle,gcf);       % Initial configuration
    hold on
    plot_robot(num2cell(Q(:,end)),L,angle,gcf);     % Final configuration
    
    legend([p1 p2 p3],'P1 pose','P2 pose','joint trajectory');
    
elseif task == 2
    p1 = plot3(S(1,1),S(2,1),S(3,1),'om');                           % First point of the shape
    hold on
    p2 = plot3(S(1,end),S(2,end),S(3,end),'o','Color','#EDB120');    % Last point of the shape
    p3 = plot3(S(1,:),S(2,:),S(3,:),'k');                            % Trajectory
    
    plot_robot(num2cell(Q(:,1)),L,angle,gcf);       % Initial configuration
    hold on
    plot_robot(num2cell(Q(:,end)),L,angle,gcf);     % Final configuration
    
    legend('P2 pose','P3 pose','joint trajectory');

elseif task == 3
    p1 = plot3(S(1,end),S(2,end),S(3,end),'o','Color','#EDB120');	% Last point of the shape
    hold on
    p2 = plot3(S_home(1),S_home(2),S_home(3),'oc');                 % Home point
    p3 = plot3(S(1,:),S(2,:),S(3,:),'k');                           % Trajectory
    
    plot_robot(num2cell(Q(:,1)),L,angle,gcf);       % Initial configuration
    hold on
    plot_robot(num2cell(Q(:,end)),L,angle,gcf);     % Final configuration
    
    legend('P3 pose','P1 pose','joint trajectory');
end

if task == 1
    task_title = " from P1 to P2 ";
elseif task == 2
    task_title = " from P2 to P3 ";
else 
    task_title = " from P3 to P1 ";
end

grid on
axis equal
title(strcat('Manipulator poses and joint trajectory',task_title));
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(30,20) % Point of view: (azimut,elevation)


% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name',strcat("Motion",task_title,"- Gripper space"),'NumberTitle','off')
t = tiledlayout(3,1);
fsz = 12;

nexttile
plot(tt,S(1,:), tt,S(2,:), tt,S(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on

nexttile
plot(tt,Sp(1,:), tt,Sp(2,:), tt,Sp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity [m/s]')
grid on

nexttile
plot(tt,Spp(1,:), tt,Spp(2,:), tt,Spp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration [m/s^2]')
grid on

leg = legend('$x$ \space $axis$','$y$ \space $axis$','$z$ \space $axis$','Orientation', 'Horizontal');
set(leg,'Interpreter','latex','FontSize',fsz)
% leg.Position = ax.Position;

title(t,strcat('Gripper coordinates',task_title));


% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name',strcat('Motion',task_title,'- Joint space'),'NumberTitle','off')
t = tiledlayout(3,1);
fsz = 10;

nexttile
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position')
grid on
h1 = legend('$q_1$ \space $[rad]$','$q_2$ \space $[m]$','$q_3$ \space $[rad]$','Location','northeastoutside');
set(h1,'Interpreter','latex','FontSize',fsz)

nexttile
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('velocity')
grid on
h2 = legend('$\dot{q_1}$ \space $[rad/s]$','$\dot{q_2}$ \space $[m/s]$','$\dot{q_3}$ \space $[rad/s]$','Location','northeastoutside');
set(h2,'Interpreter','latex','FontSize',fsz)

nexttile
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('acceleration')
grid on
h3 = legend('$\ddot{q_1}$ \space $[rad/s^2]$','$\ddot{q_2}$ \space $[m/s^{2}]$','$\ddot{q_3}$ \space $[rad/s^2]$','Location','northeastoutside');
set(h3,'Interpreter','latex','FontSize',fsz)

title(t,strcat('Joint coordinates',task_title));


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name',strcat('Motion along the trajectory',task_title,'- Curvilinear abscissa'),'NumberTitle','off')
plot(tt,pos, tt,vel, tt,acc, [tt(1) tt(end)],[0 0],'k')
title('Motion along the trajectory')
xlim([tt(1) tt(end)])
xlabel('time [s]')
grid on
leg = legend('$displacement$ \space $[m]$','$velocity$ \space $[m/s]$','$acceleration$ \space $[m/s^2]$','Location','northwest');
set(leg,'Interpreter', 'latex','FontSize',fsz)

title('Curvilinear absissa, velocity and acceleration along the trajectory');


% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);

nexttile

plot(tt,Sp(1,:), tt,xp_diff, tt,Spp(1,:), tt,xpp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('x direction')
grid on
h00 = legend('$\dot{x}$ $-$ $ analytical$ \space $[m/s]$','$\dot{x}$ $-$ $ numerical$ \space $[m/s]$',...
           '$\ddot{x}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{x}$ $-$ $ numerical$ \space $[m/s^2]$',...
           'Location','northeastoutside');
set(h00,'Interpreter','latex','FontSize',fsz)

nexttile
plot(tt,Sp(2,:), tt,yp_diff, tt,Spp(2,:), tt,ypp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('y direction')
grid on
h01 = legend('$\dot{y}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{y}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{y}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{y}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h01,'Interpreter','latex','FontSize',fsz)
   
nexttile
plot(tt,Sp(3,:), tt,zp_diff, tt,Spp(3,:), tt,zpp_diff, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('z direction')
grid on
h02 = legend('$\dot{z}$ $-$ $ analytical$ \space $[m/s]$', '$\dot{z}$ $-$ $ numerical$ \space $[m/s]$',...
            '$\ddot{z}$ $-$ $ analytical$ \space $[m/s^2]$','$\ddot{z}$ $-$ $ numerical$ \space $[m/s^2]$',...
            'Location','northeastoutside');
set(h02,'Interpreter','latex','FontSize',fsz)

title(t,{'Gripper coordinates','Analytical vs numerical solution'});


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
set(h4,'Interpreter','latex','FontSize',fsz)
   
nexttile
grid on
plot(tt,Qp(2,:), tt,Qp_diff(2,:), tt,Qpp(2,:), tt,Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 2')
grid on
h5 = legend('$\dot{q_2}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_2}$ $-$ $ numerical$ \space $[rad/s]$',...
           '$\ddot{q_2}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_2}$ $-$ $ numerical$ \space $[rad/s^2]$',...
           'Location','northeastoutside');
set(h5,'Interpreter','latex','FontSize',fsz)
   
nexttile
grid on
plot(tt,Qp(3,:), tt,Qp_diff(3,:), tt,Qpp(3,:), tt,Qpp_diff(3,:), [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('joint 3')
grid on
h6 = legend('$\dot{q_3}$ $-$ $ analytical$ \space $[rad/s]$',   '$\dot{q_3}$ $-$ $ numerical$ \space $[rad/s]$',...
          '$\ddot{q_3}$ $-$ $ analytical$ \space $[rad/s^2]$','$\ddot{q_3}$ $-$ $ numerical$ \space $[rad/s^2]$',...
          'Location','northeastoutside');
set(h6,'Interpreter','latex','FontSize',fsz)

title(t,{strcat('Joint coordinates',task_title),'Analytical vs numerical solution'}); 