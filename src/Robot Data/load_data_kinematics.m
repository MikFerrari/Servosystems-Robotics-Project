% NUMBER OF LINKS
nLinks = 3;

% LINK DIMENSIONS [m]
l1a = 0.270; l1b = 0.278; l1c = 0.620; l2 = 0.110; l3 = 0.299;

L = [l1a l1b l1c l2 l3];

% TILT ANGLE [rad]
% Fixed tilt angle of the arm
angle = pi/6;

% ACTUATOR POSITION LIMITS
q1_inf = -pi;       q1_sup = pi;        % [rad]
q2_inf = 0;         q2_sup = 0.4;       % [m]
q3_inf = -0.5*pi;   q3_sup = 0.5*pi;    % [rad]

pos_limits = [q1_inf q1_sup q2_inf q2_sup q3_inf q3_sup];

% ACTUATOR VELOCITY LIMITS
q1p_inf = -4;       q1p_sup = 4;        % [rad/s]
q2p_inf = -0.25;    q2p_sup = 0.25;     % [m/s]
q3p_inf = -8;       q3p_sup = 8;        % [rad/s]

vel_limits = [q1p_inf q1p_sup q2p_inf q2p_sup q3p_inf q3p_sup];

% ACTUATOR ACCELERATION LIMITS
% Chosen as half of the maximum velocity, but it is not mandatory
q1pp_inf = -2;      q1pp_sup = 2;       % [rad/s^2]
q2pp_inf = -0.1;    q2pp_sup = 0.1;     % [m/s^2]
q3pp_inf = -4;      q3pp_sup = 4;       % [rad/s^2]

acc_limits = [q1pp_inf q1pp_sup q2pp_inf q2pp_sup q3pp_inf q3pp_sup];