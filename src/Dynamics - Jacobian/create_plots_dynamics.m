if task == 1
    task_title = " from P1 to P2 ";
    task_tag = "\task1_";
elseif task == 2
    task_title = " from P2 to P3 ";
    task_tag = "\task2_";
else 
    task_title = " from P3 to P1 ";
    task_tag = "\task3_";
end

plot_dyn_directory = ['C:\Users\miche\OneDrive - unibs.it\University\Magistrale - UNIBS\' ...
                      '1_Primo Anno\Servosystems & Robotics\Servosystems & Robotics Project\src\Dynamics - Jacobian\plots'];
                  

% FORCES/TORQUES APPLIED BY THE ACTUATORS
figure('Name',strcat('Actuator torques and forces',task_title),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt, Fq(1,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('Nm')
title('Joint 1')

nexttile
plot(tt, Fq(2,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('N')
title('Joint 2')

nexttile
plot(tt, Fq(3,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('Nm')
title('Joint 3')

title(t,strcat('Actuator torques and forces',task_title));
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
set(gcf,'renderer','painters');
exportgraphics(gcf, strcat(plot_dyn_directory,task_tag,'ActuatorActions.pdf'))

% POWER DEVELOPED BY THE ACTUATORS
figure('Name',strcat('Actuator power',task_title),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt, WWq(1,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('W')
title('Joint 1')

nexttile
plot(tt, WWq(2,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('W')
title('Joint 2')

nexttile
plot(tt, WWq(3,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('W')
title('Joint 3')

title(t,strcat('Actuator power',task_title));
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
set(gcf,'renderer','painters');
exportgraphics(gcf, strcat(plot_dyn_directory,task_tag,'ActuatorPower.pdf'))

% KINETIC, POTENTIAL & TOTAL ENERGY
figure('Name',strcat('Kinetic and potential energy',task_title),'NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt, Ep)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('J')
title('Total Potential Energy')

nexttile
plot(tt, Ek)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('J')
title('Total Kinetic Energy')

nexttile
plot(tt, Etot)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('J')
title('Total Mechanical Energy (E_k+ E_p)')

title(t,strcat('Kinetic and potential energy',task_title));
lines = findobj(gcf,'Type','Line');
for i = 1:numel(lines)
  lines(i).LineWidth = 1.5;
end
set(gcf,'renderer','painters');
exportgraphics(gcf, strcat(plot_dyn_directory,task_tag,'KinPotMecEnergy.pdf'))

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
figure('Name',strcat('Power equality 1',task_title),'NumberTitle','off')
plot(tt(1:end-1),Etot_diff, tt,Wtot, [tt(1) tt(end)],[0 0],'k')
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('power [W]')
title(strcat('Conservation of mechanical energy',task_title))
legend('$dE_{tot}/dt$','$W_{q}+W_{ext}$','Location','northeast','Interpreter','latex')
set(gcf,'renderer','painters');
exportgraphics(gcf, strcat(plot_dyn_directory,task_tag,'EnergyConservation1.pdf'))

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name',strcat('Power equality 2',task_title),'NumberTitle','off')
plot(tt(1:end-1),Ek_diff, tt,Wtot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('power [W]')
title(strcat('Conservation of mechanical energy',task_title))
legend('$dE_{k}/dt$','$W_{q}+W_{ext}+W_{weight}$','Location','northeast','Interpreter','latex')
set(gcf,'renderer','painters');
exportgraphics(gcf, strcat(plot_dyn_directory,task_tag,'EnergyConservation2.pdf'))