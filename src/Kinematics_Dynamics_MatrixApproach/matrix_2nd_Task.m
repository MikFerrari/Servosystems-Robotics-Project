%% JACOBIAN KINEMATICS - 2ND TASK
% Kinematic study of the manipulator performing the given task
%
% 2nd task: motion along the sampled trajectory
% Points are connected in the gripper space using the spline algorithm

dT = 0.03; % Initial guess
Qp = [Inf; Inf; Inf]; Qpp = [Inf; Inf; Inf];
Q0 = num2cell(Q_initial_shape);

%% Optimization of the value of dT
%{
while sum([max(abs(Qp),[],2) > vel_limits([2 4 6])'; max(abs(Qpp),[],2) > acc_limits([2 4 6])']) > 0
    xp_shape_diff = diff(S_shape(1,:),[],2)/dT;
    yp_shape_diff = diff(S_shape(2,:),[],2)/dT;
    zp_shape_diff = diff(S_shape(3,:),[],2)/dT;
    Sp_shape_diff = [[0; 0; 0] [xp_shape_diff; yp_shape_diff; zp_shape_diff]];
%     Sp_shape_diff = [xp_shape_diff; yp_shape_diff; zp_shape_diff];

    
    xpp_shape_diff = diff(Sp_shape_diff(1,:),[],2)/dT;
    ypp_shape_diff = diff(Sp_shape_diff(2,:),[],2)/dT;
    zpp_shape_diff = diff(Sp_shape_diff(3,:),[],2)/dT;
    Spp_shape_diff = [[0; 0; 0] [xpp_shape_diff; ypp_shape_diff; zpp_shape_diff]];
%     Spp_shape_diff = [[xpp_shape_diff; ypp_shape_diff; zpp_shape_diff]];

    
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
%}

% Minimum sampling time to respect velocity and acceleration constraints of actuators
% disp(dT) 

% Minimum sampling time (ROUNDED) to respect velocity and acceleration constraints of actuators
dT = ceil(dT*100);
dT = dT/100
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

