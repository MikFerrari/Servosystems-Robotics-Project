% Compute sampling time in order to respect velocity and acceleration
% constraints of the joints
dT = 0.025; % Initial guess
Qp = [Inf; Inf; Inf]; Qpp = [Inf; Inf; Inf]; Fq = [Inf; Inf; Inf];
Q0 = num2cell(Q_initial_shape);

while sum([max(abs(Qp),[],2) > vel_limits([2 4 6])'; ...
           max(abs(Qpp),[],2) > acc_limits([2 4 6])'; ...
           max(abs(Fq),[],2) > torque_limits([2 4 6])']) > 0
       
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
        
        % CONVERT FROM SYMBOLIC TO NUMERICAL VALUES
        Je = double(subs(Je_symb,Qe_symb,[L'; angle; Q(:,i); Gpos{2}; Gpos{3}; Gpos{4}; q1; q3]));
        Jep = double(subs(Jep_symb,Qep_symb,[L'; angle; Q(:,i); Qp(:,i); Gpos{2}; Gpos{3}; Gpos{4}; q1; q3]));
        
        % ACTUATOR TORQUES
        Fq(:,i) = (Je'*M*Je)*Qpp(:,i)+Je'*M*Jep*Qp(:,i)-Je'*Ftot;
        
    end
    
    if sum([max(abs(Qp),[],2) > vel_limits([2 4 6])'; ...
            max(abs(Qpp),[],2) > acc_limits([2 4 6])'; ...
            max(abs(Fq),[],2) > torque_limits([2 4 6])']) == 0
        break
    end
    
    dT = 1.1*dT;
    
end

% Minimum sampling time (ROUNDED) to respect velocity and acceleration constraints of actuators
dT = ceil(dT*100);
dT = dT/100;