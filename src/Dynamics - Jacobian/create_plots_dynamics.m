% FORCES/TORQUES APPLIED BY THE ACTUATORS
figure('Name','Actuator torques and forces','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt, Fq(1,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('torque [Nm]')
title('Joint 1')

nexttile
plot(tt, Fq(2,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('force [N]')
title('Joint 2')

nexttile
plot(tt, Fq(3,:))
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('torque [Nm]')
title('Joint 3')

title(t,'Actuator torques and forces');


% KINETIC, POTENTIAL & TOTAL ENERGY
figure('Name','Kinetic and potential energy','NumberTitle','off')
t = tiledlayout(3,1);

nexttile
plot(tt, Ep)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('energy [J]')
title('Total Potential Energy')

nexttile
plot(tt, Ek)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('energy [J]')
title('Total Kinetic Energy')

nexttile
plot(tt, Etot)
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('energy [J]')
title('Total Mechanical Energy (E_k+E_p)')

title(t,'Kinetic and potential energy');


% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
figure('Name','Power equality 1','NumberTitle','off')
plot(tt(1:end-1),Etot_diff, tt,Wtot, [tt(1) tt(end)],[0 0],'k')
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('power [W]')
title('Conservation of mechanical energy (formulation 1)')
legend('dE_{tot}/dt','W_{q}+W_{ext}','Location','northeast')

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name','Power equality 2','NumberTitle','off')
plot(tt(1:end-1),Ek_diff, tt,Wtot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
xlim([tt(1) tt(end)])
xlabel('time [s]'), ylabel('power [W]')
title('Conservation of mechanical energy (formulation 2)')
legend('dE_{k}/dt','W_{q}+W_{ext}+W_{weight}','Location','northeast')