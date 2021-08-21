function [x,xp,xpp] = cycloidal(t,T,S0,dS)
%
% Cycloidal Motion Law (Sinusoidal Acceleration)
%
% t --> current time
% T --> total time
% S0 --> starting position
% dS --> total motion
% 
% Assume Vini = Vfin = 0

    A = dS*2*pi/T^2;
    V = A*T/(2*pi);
    
    x = -A*(T/(2*pi))^2*sin(2*pi/T*t)+A*T/(2*pi)*t+S0;
    xp = -A*T/(2*pi)*cos(2*pi/T*t)+V;
    xpp = A*sin(2*pi/T*t);

end