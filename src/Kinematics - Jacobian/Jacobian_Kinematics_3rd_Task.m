%% JACOBIAN KINEMATICS - 3RD TASK
% Kinematic study of the manipulator performing the given task
%
% 3rd task: point-to-point motion
% From the last point of the shape back to home with minimum actuation time
% and cycloidal law of motion


%% Preliminary operations

% LOAD ROBOT DATA
clc; clear; close all
load_robot_data;


% COMPUTE TRAJECTORY FOR THE 2ND TASK
nPoints_spline = 100;   dilation_factor = 0.35;
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

Q = zeros(3,nPoints);   Qp = zeros(3,nPoints);  Qpp = zeros(3,nPoints);
S = zeros(3,nPoints);   Sp = zeros(3,nPoints);  Spp = zeros(3,nPoints);
pos = zeros(1,nPoints);

tt = linspace(0,Ttot,nPoints);


%% Loop through each sampling period

for i = 1:nPoints
    
    % GRIPPER COORDINATES
    [q1,q1p,q1pp] = cycloidal(tt(i),Ttot,Q_final_shape(1),dQ(1));
    [q2,q2p,q2pp] = cycloidal(tt(i),Ttot,Q_final_shape(2),dQ(2));
    [q3,q3p,q3pp] = cycloidal(tt(i),Ttot,Q_final_shape(3),dQ(3));
    
    Q(:,i) = real([q1; q2; q3]);
    Qp(:,i) = real([q1p; q2p; q3p]);
    Qpp(:,i) = real([q1pp; q2pp; q3pp]);
    
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


figure('name','Motion from the last point of the shape back to home','NumberTitle','off')
plot3(S(1,:),S(2,:),S(3,:))              % Trajectory
hold on
plot3(S_final_shape(1),S_final_shape(2),S_final_shape(3),'og')    % Last point of the shape
plot3(S_home(1),S_home(2),S_home(3),'or')                         % Home point
grid on
view(20,65)
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
title('Motion from the home position to the beginning of the shape');
legend('trajectory','last point of the shape','home position');


%% Compute derivatives (numerical differentiation) to debug results

dT = Ttot/nPoints;
diff_debug_kin


%% Plot all results

task = 3;
create_plots_kinematics