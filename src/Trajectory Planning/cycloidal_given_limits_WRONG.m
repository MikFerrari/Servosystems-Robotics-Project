function [x,xp,xpp] = cycloidal_given_limits(t,pos_lim,vel_lim,acc_lim,S0,dS)
%
% Cycloidal Motion Law (Sinusoidal Acceleration)
%
% t --> current time
% pos_lim --> maximum position (2 values, sup and inf)
% vel_lim --> maximum velocity (absolute value)
% acc_lim --> maximum acceleration (absolute value)
% S0 --> starting position
% dS --> total motion
% 
% Assume Vini = Vfin = 0

    A = acc_lim;

    while V > vel_lim || x < pos_lim(1) || x > pos_lim(2)
        T = sqrt(2*pi*dS/A);
        V = A*T/(2*pi);

        x = -A*(T/(2*pi))^2*sin(2*pi/T*t)+A*T/(2*pi)*t+S0;
        xp = -A*T/(2*pi)*cos(2*pi/T*t)+V;
        xpp = A*sin(2*pi/T*t);
        
        A = 0.95*A;
    end

end