%%
% plot_control_directory = 'C:\Users\monti\OneDrive - unibs.it\Servosystems & Robotics Project\src\Control\control_plot';
plot_control_directory = 'C:\Users\monti\OneDrive - unibs.it\Servosystems & Robotics Project\src\Control\plot_not_controlled';

%% Gripper space
figure('name',strcat("Motion",task_title,"- Gripper position"),'NumberTitle','off')
fsz=12;
plot(tt,x_grip,out.tout,out.x_grip_result,tt,y_grip,out.tout,out.y_grip_result,tt,z_grip,out.tout,out.z_grip_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('x ideal','x actual','y ideal','y actual','z ideal','z actual','Location','southwest')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Gripper position',task_title));
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_x_gripper.pdf'))

figure('name',strcat("Motion",task_title,"- Gripper velocity"),'NumberTitle','off')
plot(tt,xp_grip,out.tout,out.xp_grip_result,tt,yp_grip,out.tout,out.yp_grip_result,tt,zp_grip,out.tout,out.zp_grip_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('$\dot{x}$ ideal','$\dot{x}$ actual','$\dot{y}$ ideal','$\dot{y}$ actual','$\dot{z}$ ideal','$\dot{z}$ actual','Location','southwest')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Gripper velocity',task_title));
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_y_gripper.pdf'))

figure('name',strcat("Motion",task_title,"- Gripper acceleration"),'NumberTitle','off')
plot(tt,xpp_grip,out.tout,out.xpp_grip_result,tt,ypp_grip,out.tout,out.ypp_grip_result,tt,zpp_grip,out.tout,out.zpp_grip_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('$\ddot{x}$ ideal','$\ddot{x}$ actual','$\ddot{y}$ ideal','$\ddot{y}$ actual','$\ddot{z}$ ideal','$\ddot{z}$ actual','Location','southeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title('Gripper acceleration')
grid on
hold off
title(strcat('Gripper acceleration',task_title));
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_z_gripper.pdf'))


%% Joints space
figure('name',strcat("Motion",task_title,"- Joints position"),'NumberTitle','off')

fsz=12;
plot(tt,Q(1,:),out.tout,out.q1_result,tt,Q(2,:),out.tout,out.q2_result,tt,Q(3,:),out.tout,out.q3_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('$q_1$ ideal','$q_1$ actual','$q_2$ ideal','$q_2$ actual','$q_3$ ideal','$q_3$ actual','Location','northeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Joints position',task_title));
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_x_joints.pdf'))

figure('name',strcat("Motion",task_title,"- Joints velocity"),'NumberTitle','off')
plot(tt,Qp(1,:),out.tout,out.q1p_result,tt,Qp(2,:),out.tout,out.q2p_result,tt,Qp(3,:),out.tout,out.q3p_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('$\dot{q}_1$ ideal','$\dot{q}_1$ actual','$\dot{q}_2$ ideal','$\dot{q}_2$ actual','$\dot{q}_3$ ideal','$\dot{q}_3$ actual','Location','northeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Joints velocity',task_title));
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_y_joints.pdf'))

figure('name',strcat("Motion",task_title,"- Joints acceleration"),'NumberTitle','off')
plot(tt,Qpp(1,:),out.tout,out.q1pp_result,tt,Qpp(2,:),out.tout,out.q2pp_result,tt,Qpp(3,:),out.tout,out.q3pp_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('$\ddot{q}_1$ ideal','$\ddot{q}_1$ actual','$\ddot{q}_2$ ideal','$\ddot{q}_2$ actual','$\ddot{q}_3$ ideal','$\ddot{q}_3$ actual','Location','southeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Joints acceleration',task_title));
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_z_joints.pdf'))

%% Joints forces
figure('name',strcat("Motion",task_title,"- Joints torque 1"),'NumberTitle','off')
plot(tt,Fq(1,:),out.tout,out.torque1_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('Torque 1 ideal','Torque 1 actual','Location','southeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat("Joints torque 1 ",task_title))
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_torque1.pdf'))

figure('name',strcat("Motion",task_title,"- Joints force 2"),'NumberTitle','off')
plot(tt,Fq(2,:),out.tout,out.force2_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('Force 2 ideal','Force 2 actual','Location','southeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Joints force 2 ',task_title))
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_force2.pdf'))

figure('name',strcat("Motion",task_title,"- Joints torque 3"),'NumberTitle','off')
plot(tt,Fq(3,:),out.tout,out.torque3_result,'LineWidth',1.5)
hold on
xlim([tt(1) tt(end)])
xlabel('time [s]'); ylabel('position [m]')
leg = legend('Torque 3 ideal','Torque 3 actual','Location','southeast')
set(leg, 'Interpreter', 'latex','FontSize',fsz)
title(strcat('Joints torque 3 ',task_title))
grid on
hold off
exportgraphics(gcf, strcat(plot_control_directory,'\task',num2str(task),'_torque3.pdf'))