%% JACOBIAN DYNAMICS - 1ST TASK
% Dynamic study of the manipulator performing the given task
%
% 3rd task: point-to-point motion
% From the last point of the shape back to home with minimum actuation time
% and cycloidal law of motion


%% Preliminary operations

% LOAD ROBOT DATA
% clc; clear
load_robot_data;


% COMPUTE TRAJECTORY FOR THE 2ND TASK
nPoints_spline = 100;   dilation_factor = 0.35;
x_tilt = deg2rad(45);   y_tilt = deg2rad(65);   z_tilt = deg2rad(10);
x_offset = 0.95;        y_offset = 0.3;         z_offset = 0.8;

spline_points_target = compute_trajectory(nPoints_spline,dilation_factor, ...
                                          x_tilt,y_tilt,z_tilt, ...
                                          x_offset,y_offset,z_offset);
                                      
                                      
% EVALUATE SYMBOLIC EXTENDED JACOBIAN AND ITS TEMPORAL DERIVATIVE
syms l1a l1b l1c l2 l3 tilt_angle ...
     q1_sym(t) q2_sym(t) q3_sym(t) ...
     q1p_sym(t) q2p_sym(t) q3p_sym(t)
syms G1 [1 3]
syms G2 [1 3]
syms G3 [1 3]
syms q1 q3
Qsymb = [l1a; l1b; l1c; l2; l3; tilt_angle; ...
         q1_sym(t); q2_sym(t); q3_sym(t); ...
         G1.'; G2.'; G3.'; ...
         q1; q3];
Qp_symb = [l1a; l1b; l1c; l2; l3; tilt_angle; ...
           q1_sym(t); q2_sym(t); q3_sym(t); ...
           q1p_sym(t); q2p_sym(t); q3p_sym(t); ...
           G1.'; G2.'; G3.';
           q1; q3];

[~,~,Je_symb] = jac_ext_symb;
[~,~,Jep_symb] = jacP_ext_symb;   


% HOME AND TARGET POSITIONS
% Final point
Q_home = {0; 0; 0};
[S_home,~] = dir_kin(Q_home,L,angle);

S_home = cell2mat(S_home);
S_final_shape = spline_points_target(1:3,end);

Q_home = cell2mat(Q_home);

% Starting point
Q0_final_shape = inv_kin(num2cell(S_final_shape),L,angle,1);
Q_final_shape = cell2mat(inv_kin_numerical(num2cell(S_final_shape),L,angle,Q0_final_shape));

dQ = Q_home-Q_final_shape;
abs_dQ = norm(dQ);


% ACTUATION TIME
% Formula for the minimum actuation time in the case of cycloidal law of motion
T1_min = max([2*abs_dQ/vel_limits(2) sqrt(2*pi*abs_dQ/acc_limits(2)) sqrt(2*pi*abs_dQ/abs(acc_limits(1)))]);
T2_min = max([2*abs_dQ/vel_limits(4) sqrt(2*pi*abs_dQ/acc_limits(4)) sqrt(2*pi*abs_dQ/abs(acc_limits(3)))]);
T3_min = max([2*abs_dQ/vel_limits(6) sqrt(2*pi*abs_dQ/acc_limits(6)) sqrt(2*pi*abs_dQ/abs(acc_limits(5)))]);

% The laws are computed for the whole duration of the motion of the slowest joint
Ttot = max([T1_min T2_min T3_min]);


% INITIALIZE ARRAYS
% Number of points in which to compute the law of motion
nPoints = 100;

nExtCoord = size(formula(Je_symb),1);

Q = zeros(3,nPoints); Qp = zeros(3,nPoints); Qpp = zeros(3,nPoints);

S = zeros(nExtCoord,nPoints); Sp = zeros(nExtCoord,nPoints); Spp = zeros(nExtCoord,nPoints);

pos = zeros(1,nPoints);

Fq = zeros(3,nPoints);

Ek = zeros(1,nPoints); Ep = zeros(1,nPoints); Etot = zeros(1,nPoints);

Wq = zeros(1,nPoints); Wext = zeros(1,nPoints); Wweight = zeros(1,nPoints);
Wtot = zeros(1,nPoints);
Wtot_plus_weight = zeros(1,nPoints);
WWq = zeros(3,nPoints);

% MASSES OF THE LINKS
% 5-mass approximation for link 1
% Jg_links{2}(3) --> moment of inertia around x axiz (rotation axis) of
%                    frame attached to link 3
G_ref0 = [L(2); 0; L(1)]+Gpos{2};

A = [1      1      1                     1         1;
     0      0      G_ref0(1)             L(2)      L(2);
     0      0      G_ref0(2)             0        -L(3)*cos(angle);
     0      L(1)   G_ref0(3)             L(1)      (L(1)+L(3)*sin(angle));
     0      0      norm(G_ref0(1:2))^2   L(2)^2    (L(2)^2+(L(3)*cos(angle))^2)];

B = [m1; m1*G_ref0; Jg_links{2}(3)];

X = A\B;

m1_prox = X(1); m1_a = X(2); m1_g = X(3); m1_b = X(4); m1_dist = X(5);

% 3-mass approximation for link 2
% Jg_links{3}(1) --> moment of inertia around x axiz (rotation axis) of
%                    frame attached to link 3
% Gpos{3}(2) --> position of the CoM of link 3 wrt frame attached to link 3
%                along the y-axis of the same frame

Jg2_matrix = [Jg_links{3}(1) Jg_links{3}(1) Jg_links{3}(1);
              Jg_links{3}(1) Jg_links{3}(1) Jg_links{3}(1);
              Jg_links{3}(1) Jg_links{3}(1) Jg_links{3}(1)];
           
Jg2_matrix = [Jg2_matrix   [0; 0; 0]
                [0 0 0]        m2   ];

M_inclinedToVert = eye(4);
M_inclinedToVert(1:2,1:2) = [cos(angle) -sin(angle);
                             sin(angle)  cos(angle)];
Jg2_vert_axis = M_inclinedToVert*Jg2_matrix*M_inclinedToVert';

m2_dist = Jg2_vert_axis(1,1)/((L(4)-abs(Gpos{3}(2)))*L(4));
m2_prox = Jg2_vert_axis(1,1)/(abs(Gpos{3}(2))*L(4));
m2_g = m2-Jg2_vert_axis(1,1)/(abs(Gpos{3}(2))*(L(4)-abs(Gpos{3}(2))));

% 3-mass approximation for link 3
% Jg_links{4}(2) --> moment of inertia around y axiz (rotation axis) of
%                    frame attached to link 3
% Gpos{4}(1) --> position of the CoM of link 3 wrt frame attached to link 3
%                along the x-axis of the same frame
m3_dist = Jg_links{4}(2)/((L(5)-abs(Gpos{4}(1)))*L(5));
m3_prox = Jg_links{4}(2)/(abs(Gpos{4}(1))*L(5));
m3_g = m3-Jg_links{4}(2)/(abs(Gpos{4}(1))*(L(5)-abs(Gpos{4}(1))));


% MASS MATRIX
M = diag([repelem(m_grip+m3_dist,3), ...
          repelem(m3_g,3), ...
          repelem(m3_prox+m2_dist,3), ...
          repelem(m2_g,3), ...
          repelem(m2_prox+m1_dist,3), ...
          repelem(m1_b,3), ...
          repelem(m1_g,3), ...
          repelem(m1_a,3), ...
          (Jg_links{2}(3)+Jg2_vert_axis(1,1)), Jg_links{4}(2)]);

      
% EXTERNAL FORCES AND WEIGHT MATRIX
Fext = [fx; fy; fz; zeros(size(M,1)-3,1)];

g = -9.81;
Ag = [repmat([0; 0; g],(nExtCoord-2)/3,1); [0; 0]];

Ftot = Fext+M*Ag;


tt = linspace(0,Ttot,nPoints);


%% Loop through each sampling period

for i = 1:nPoints
    
    %%%%% KINEMATICS %%%%%
    
    % GRIPPER COORDINATES
    [q1,q1p,q1pp] = cycloidal(tt(i),Ttot,Q_final_shape(1),dQ(1));
    [q2,q2p,q2pp] = cycloidal(tt(i),Ttot,Q_final_shape(2),dQ(2));
    [q3,q3p,q3pp] = cycloidal(tt(i),Ttot,Q_final_shape(3),dQ(3));
    
    Q(:,i) = real([q1; q2; q3]);
    Qp(:,i) = real([q1p; q2p; q3p]);
    Qpp(:,i) = real([q1pp; q2pp; q3pp]);
    
    % COORDINATES THROUGH DIRECT KINEMATICS
    [~,~,Scell] = dir_kin_ext(num2cell(Q(:,i)),L,angle,Gpos);
    S(:,i) = cell2mat(Scell);

    % CONVERT FROM SYMBOLIC TO NUMERICAL VALUES
    Je = double(subs(Je_symb,Qsymb,[L'; angle; Q(:,i); Gpos{2}; Gpos{3}; Gpos{4}; q1; q3]));
    Jep = double(subs(Jep_symb,Qp_symb,[L'; angle; Q(:,i); Qp(:,i); Gpos{2}; Gpos{3}; Gpos{4}; q1; q3]));
    
    % VELOCITY ARRAY
    Sp(:,i) = Je*Qp(:,i);    
    
    % ACCELERATION ARRAY
    Spp(:,i) = Je*Qpp(:,i)+Jep*Qp(:,i);
    
    % POSITION OF THE GRIPPER ALONG THE TRAJECTORY
    % (velocity and acceleration are computed afterwards using diff()/dT)
    if i ~= 1
        pos(i) = pos(i-1) + norm(S(1:3,i) - S(1:3,i-1));
    elseif i == 1
        pos(i) = 0;
    end
    
    %%%%% END KINEMATICS %%%%%
    
    
    %%%%% DYNAMICS %%%%%
    
    % ACTUATOR TORQUES
    Fq(:,i) = (Je'*M*Je)*Qpp(:,i)+Je'*M*Jep*Qp(:,i)-Je'*Ftot;
    
    % KINETIC ENERGY
    Sp(:,i) = Je*Qp(:,i);
    Ek(:,i) = 0.5*Sp(:,i)'*M*Sp(:,i);
    
    % POTENTIAL ENERGY
    Ep(:,i) = -Ag'*M*S(:,i); % "-" since Gpos was defined as a negative value
    
    % TOTAL ENERGY
    Etot(:,i) = Ek(:,i)+Ep(:,i);
    
    % MOTOR POWER
    Wq(:,i) = Fq(:,i)'*Qp(:,i);
    WWq(:,i) = [Fq(1,i)'*Qp(1,i); Fq(2,i)'*Qp(2,i); -Fq(3,i)'*Qp(3,i)];
    
    % EXTERNAL FORCES POWER
    Wext(:,i) = Fext'*Sp(:,i);    
    
    % WEIGHT FORCES POWER
    Wweight(:,i) = (M*Ag)'*Sp(:,i);  
    
    % TOTAL POWER FROM FORCES AND TORQUES
    Wtot(:,i) = Wq(:,i)+Wext(:,i);
    Wtot_plus_weight(:,i) = Wtot(:,i)+Wweight(:,i);
    
    %%%%% END DYNAMICS %%%%%
    
end

Fq(3,:) = -Fq(3,:);

%% Compute derivatives (numerical differentiation) to debug results

dT = Ttot/nPoints;

diff_debug_kin

% DERIVATIVE OF TOTAL ENERGY and JUST KINETIC ENERGY
Etot_diff = diff(Etot)/dT;
Ek_diff = diff(Ek)/dT;


%% Plot all results

% PLOTS
task = 3;
create_plots_kinematics
create_plots_dynamics


%% Set data for Simulink simulation

interpolate_Q_T