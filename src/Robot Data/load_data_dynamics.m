% ACTUATOR FORCE/TORQUE LIMITS
T1_inf = -1000;     T1_sup = 1000;      % [Nm]
T2_inf = -200;      T2_sup = 200;       % [N]
T3_inf = -500;      T3_sup = 500;       % [Nm]

torque_limits = [T1_inf T1_sup T2_inf T2_sup T3_inf T3_sup];


% Script that outputs the struct smiData, containing the inertial properties of the links
robot_assembly_DataFile;

% Find specific link in smiData structure
idx_link1 = contains({smiData.Solid(:).ID},'_1');
idx_link2 = contains({smiData.Solid(:).ID},'_2');
idx_link3 = contains({smiData.Solid(:).ID},'_3');


% POSITION OF THE CENTRES OF MASS
% Distance from DISTAL joint (origin of the frame attached to the considered link) [m]

% Format: 3D arrays (spatial position of CoM) --> [xg; yg; zg];

g0 = nan;
G0 = NaN(3,1);

% HARD-CODED FOR THE SAKE OF CONVENIENCE (See PDF File: Link1 Inertial Data)
G1 = [-0.179; -0.078; -0.04];

g2_ending = smiData.Solid(7).CoM(2)/100;
m2_ending = smiData.Solid(7).mass;
g2_piston = (smiData.Solid(1).CoM(2) + ...
            (smiData.RigidTransform(12).translation(2)-smiData.RigidTransform(11).translation(2))/10)/100;
m2_piston = smiData.Solid(1).mass;
g2 = (g2_ending*m2_ending+g2_piston*m2_piston)/(m2_ending+m2_piston);

G2 = [0; g2; 0];

g3 = smiData.Solid(9).CoM(1)/100;

G3 = [g3; 0; 0];

% Gpos = {g0 G1 l1c+l2+q2-g2 -l3+abs(g3)};
Gpos = {G0 G1 G2 G3};




% MASSES [kg]
m0 = nan;
m1 = sum([smiData.Solid(idx_link1).mass]);
m2 = sum([smiData.Solid(idx_link2).mass]);
m3 = sum([smiData.Solid(idx_link3).mass]);

M_links = [m0 m1 m2 m3];


% INERTIA MOMENTS (wrt a frame located in the barycentre of the
%                  considered link and aligned with the frame attached to that link) [kg*m^2]
% They follow Solidwork's convention (See PDF File: Solidworks Convention Inertial Data)

% Format: 6D arrays (3 MoI and 3 PoI) --> [Jxx Jyy Jzz Jxy Jxz Jyz]

Jg0 = [nan nan nan nan nan nan];

% HARD-CODED FOR THE SAKE OF CONVENIENCE (See PDF File: Link1 Inertial Data)
Jg1 = [5.873 7.234 4.887 -1.215 -2.015 -1.401];

% HARD-CODED FOR THE SAKE OF CONVENIENCE (See PDF File: Link2 Inertial Data)
Jg2 = [0.524 0.016 0.523 0 0 0]; % The 3 PoI are set to 0, they are negligible

Jg3 = [smiData.Solid(9).MoI/10000 0 0 0]; % The 3 PoI are set to 0, they are negligible

Jg_links = {Jg0 Jg1 Jg2 Jg3};


% EXTERNAL FORCES AND TORQUES ON THE GRIPPER [N and Nm]
forceValue = 0;
fx = forceValue; fy = forceValue; fz = forceValue;
Force = [fx; fy; fz];

torqueValue = 0;
cx = torqueValue; cy = torqueValue; cz = torqueValue;
Torque = [cx; cy; cz];

% MASS OF THE GRIPPER (of the object in the gripper)
m_grip = 0;