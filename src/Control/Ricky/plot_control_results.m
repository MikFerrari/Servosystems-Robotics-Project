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

%% Gripper position 
figure('name',strcat("Motion",task_title,"- Gripper space"),'NumberTitle','off')
t = tiledlayout(3,1);
fsz=12;

nexttile
plot(tt,x_grip, [tt(1) tt(end)],[0 0],'k')
hold on
plot(out.x_grip_result)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on
hold off

nexttile
plot(tt,y_grip, [tt(1) tt(end)],[0 0],'k')
hold on
plot(out.y_grip_result)
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on
hold off

nexttile
plot(tt,z_grip,  tt,out.z_grip_result, [tt(1) tt(end)],[0 0],'k')
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
grid on

leg = legend('$x$ \space $axis$','$y$ \space $axis$','$z$ \space $axis$','Orientation', 'Horizontal');
set(leg, 'Interpreter', 'latex','FontSize',fsz)
leg.Layout.Tile = 'south';

title(t,strcat('Gripper coordinates',task_title));


