%% JACOBIAN KINEMATICS - 2ND TASK
% Kinematic study of the manipulator performing the given task
%
% 2nd task: motion along the sampled trajectory
% Points are connected in the gripper space using the spline algorithm


%% Preliminary operations

% LOAD ROBOT DATA
clc; clear; close all
load_robot_data;


% COMPUTE TRAJECTORY FOR THE 2ND TASK
nPoints_spline = 200;   dilation_factor = 0.35;
x_tilt = deg2rad(45);   y_tilt = deg2rad(65);   z_tilt = deg2rad(10);
x_offset = 0.95;        y_offset = 0.3;         z_offset = 0.8;

spline_points_target = compute_trajectory(nPoints_spline,dilation_factor, ...
                                          x_tilt,y_tilt,z_tilt, ...
                                          x_offset,y_offset,z_offset);

                                      
% EVALUATE SYMBOLIC JACOBIAN AND ITS TEMPORAL DERIVATIVE
syms l1a l1b l1c l2 l3 tilt_angle ...
     q1_sym(t) q2_sym(t) q3_sym(t) ...
     q1p_sym(t) q2p_sym(t) q3p_sym(t)
Qsymb = [l1a; l1b; l1c; l2; l3; tilt_angle; ...
         q1_sym(t); q2_sym(t); q3_sym(t)];
Qp_symb = [l1a; l1b; l1c; l2; l3; tilt_angle; ...
           q1_sym(t); q2_sym(t); q3_sym(t); ...
           q1p_sym(t); q2p_sym(t); q3p_sym(t)];

[J_symb,~] = jac_symb;
[Jp_symb,~] = jacP_symb;


%% Compute the minimum sampling time to respect kinematic limits

S_initial_shape = spline_points_target(1:3,1);

Q0_initial_shape = inv_kin(num2cell(S_initial_shape),L,angle,1);
Q_initial_shape = cell2mat(inv_kin_numerical(num2cell(S_initial_shape),L,angle,Q0_initial_shape));

S_shape = [spline_points_target(1:3,1) spline_points_target(1:3,:) spline_points_target(1:3,end);]; % To assure start and end null velocity

% Number of points in which to compute the law of motion --> is the number
% of interpolation points obtained with the spline
nPoints = size(S_shape,2);

% Compute sampling time in order to respect velocity and acceleration
% constraints of the joints
dT = 0.05; % Initial guess
Qp = [Inf; Inf; Inf]; Qpp = [Inf; Inf; Inf];
Q0 = num2cell(Q_initial_shape);

while sum([max(abs(Qp),[],2) > vel_limits([2 4 6])'; max(abs(Qpp),[],2) > acc_limits([2 4 6])']) > 0
    xp_shape_diff = diff(S_shape(1,:),[],2)/dT;
    yp_shape_diff = diff(S_shape(2,:),[],2)/dT;
    zp_shape_diff = diff(S_shape(3,:),[],2)/dT;
    Sp_shape_diff = [[0; 0; 0] [xp_shape_diff; yp_shape_diff; zp_shape_diff]];
    
    xpp_shape_diff = diff(Sp_shape_diff(1,:),[],2)/dT;
    ypp_shape_diff = diff(Sp_shape_diff(2,:),[],2)/dT;
    zpp_shape_diff = diff(Sp_shape_diff(3,:),[],2)/dT;
    Spp_shape_diff = [[0; 0; 0] [xpp_shape_diff; ypp_shape_diff; zpp_shape_diff]];
    
    for i = 1:nPoints    
        
        % JOINT COORDINATES THROUGH INVERSE KINEMATICS
        Qcell = inv_kin_numerical(num2cell(S_shape(1:3,i)),L,angle,Q0);
        Q(:,i) = cell2mat(Qcell);
        Q0 = num2cell(Q(:,i));
        
        % CONVERT FROM SYMBOLIC TO NUMERICAL VALUES
        J = double(subs(J_symb,Qsymb,[L'; angle; Q(:,i)]));
    
        Qp(:,i) = J^-1*Sp_shape_diff(:,i);
        
        Jp = double(subs(Jp_symb,Qp_symb,[L'; angle; Q(:,i); Qp(:,i)]));
        
        Qpp(:,i) = J^-1*(Spp_shape_diff(:,i)-Jp*Qp(:,i));
        
    end
    
    if sum([max(abs(Qp),[],2) > vel_limits([2 4 6])'; max(abs(Qpp),[],2) > acc_limits([2 4 6])']) == 0
        break
    end
    
    dT = 1.1*dT;
end

% Minimum sampling time to respect velocity and acceleration constraints of actuators
% disp(dT) 

% Minimum sampling time (ROUNDED) to respect velocity and acceleration constraints of actuators
dT = ceil(dT*100);
dT = dT/100;
% disp(dT)

                       
%% Loop through each sampling period

% VELOCITY AND ACCELERATION ALONG THE TRAJECTORY
xp_shape_diff = diff(S_shape(1,:),[],2)/dT;
yp_shape_diff = diff(S_shape(2,:),[],2)/dT;
zp_shape_diff = diff(S_shape(3,:),[],2)/dT;
Sp_shape_diff = [[0; 0; 0] [xp_shape_diff; yp_shape_diff; zp_shape_diff]];

xpp_shape_diff = diff(Sp_shape_diff(1,:),[],2)/dT;
ypp_shape_diff = diff(Sp_shape_diff(2,:),[],2)/dT;
zpp_shape_diff = diff(Sp_shape_diff(3,:),[],2)/dT;
Spp_shape_diff = [[0; 0; 0] [xpp_shape_diff; ypp_shape_diff; zpp_shape_diff]];


% INITIALIZE ARRAYS
Q = zeros(3,nPoints);   Qp = zeros(3,nPoints);  Qpp = zeros(3,nPoints);
S = zeros(3,nPoints);   Sp = zeros(3,nPoints);  Spp = zeros(3,nPoints);
pos = zeros(1,nPoints);

tt = (0:(nPoints-1))*dT;
Q0 = num2cell(Q_initial_shape);

for i = 1:nPoints 
        
    % JOINT COORDINATES THROUGH INVERSE KINEMATICS
    Qcell = inv_kin_numerical(num2cell(S_shape(:,i)),L,angle,Q0);
    Q(:,i) = cell2mat(Qcell);
    Q0 = num2cell(Q(:,i));

    % CONVERT JACOBIAN FROM SYMBOLIC TO NUMERICAL VALUES
    J = double(subs(J_symb,Qsymb,[L'; angle; Q(:,i)]));
    
    % VELOCITY ARRAY
    Qp(:,i) = J^-1*Sp_shape_diff(:,i);
    
    % CONVERT DERIVATIVE OF JACOBIAN FROM SYMBOLIC TO NUMERICAL VALUES
    Jp = double(subs(Jp_symb,Qp_symb,[L'; angle; Q(:,i); Qp(:,i)]));

    % ACCELERATION ARRAY
    Qpp(:,i) = J^-1*(Spp_shape_diff(:,i)-Jp*Qp(:,i));
    
     % COORDINATES THROUGH DIRECT KINEMATICS
    [Scell,~] = dir_kin(num2cell(Q(:,i)),L,angle);
    S(:,i) = cell2mat(Scell);

    % CONVERT FROM SYMBOLIC TO NUMERICAL VALUES
    J = double(subs(J_symb,Qsymb,[L'; angle; Q(:,i)]));
    Jp = double(subs(Jp_symb,Qp_symb,[L'; angle; Q(:,i); Qp(:,i)]));
    
    % VELOCITY ARRAY
    Sp(:,i) = J*Qp(:,i);    
    
    % ACCELERATION ARRAY
    Spp(:,i) = J*Qpp(:,i)+Jp*Qp(:,i);
    
    % POSITION OF THE GRIPPER ALONG THE TRAJECTORY
    % (velocity and acceleration are computed afterwards using diff()/dT)
    if i ~= 1
        pos(i) = pos(i-1) + norm(S(1:3,i) - S(1:3,i-1));
    elseif i == 1
        pos(i) = 0;
    end
end


%% Compute derivatives (numerical differentiation) to debug results

% GRIPPER COORDINATES
xp_diff = [0 diff(S(1,:),[],2)/dT];
yp_diff = [0 diff(S(2,:),[],2)/dT];
zp_diff = [0 diff(S(3,:),[],2)/dT];

xpp_diff = [0 diff(Sp(1,:),[],2)/dT];
ypp_diff = [0 diff(Sp(2,:),[],2)/dT];
zpp_diff = [0 diff(Sp(3,:),[],2)/dT];

% GRIPPER VELOCITY AND ACCELERATION ALONG THE TRAJECTORY
vel = [0 diff(pos)/dT];
acc = [0 diff(vel)/dT];

% JOINT COORDINATES
Qp_diff = [[0; 0; 0] diff(Q,[],2)/dT];
Qpp_diff = [[0; 0; 0] diff(Qp,[],2)/dT];


%% Plot all results

% TRAJECTORY
figure('name','Motion along the shape - Trajectory','NumberTitle','off')
plot3(S(1,:),S(2,:),S(3,:))              % Trajectory
grid on
axis equal
view(30,20)
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
title('Trajectory along the predetermined shape');

% PLOTS
create_plots_kinematics