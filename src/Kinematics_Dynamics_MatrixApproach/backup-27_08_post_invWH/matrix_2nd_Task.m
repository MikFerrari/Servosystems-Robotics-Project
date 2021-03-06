%% JACOBIAN KINEMATICS - 2ND TASK
% Kinematic study of the manipulator performing the given task
%
% 2nd task: motion along the sampled trajectory
% Points are connected in the gripper space using the spline algorithm

dT = 0.12; % Initial guess
Qp = [Inf; Inf; Inf]; Qpp = [Inf; Inf; Inf];
Q0 = num2cell(Q_initial_shape);
%% Preliminary 
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

% tt = (0:(nPoints-1))*dT;
% Q0 = num2cell(Q_initial_shape);

%{
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
    pos(i) = norm(S(1:3,i) - S(1:3,1));
    
end


%% Compute derivatives (numerical differentiation) to debug results

% GRIPPER COORDINATES
xp_diff = diff(S(1,:),[],2)/dT;
yp_diff = diff(S(2,:),[],2)/dT;
zp_diff = diff(S(3,:),[],2)/dT;

xpp_diff = diff(Sp(1,:),[],2)/dT;
ypp_diff = diff(Sp(2,:),[],2)/dT;
zpp_diff = diff(Sp(3,:),[],2)/dT;

% GRIPPER VELOCITY AND ACCELERATION ALONG THE TRAJECTORY
vel = diff(pos)/dT;
acc = diff(vel)/dT;

% JOINT COORDINATES
Qp_diff = diff(Q,[],2)/dT;
Qpp_diff = diff(Qp,[],2)/dT;


%% Plot all results

% TRAJECTORY
figure('name','Motion along the shape - Trajectory','NumberTitle','off')
plot3(S(1,:),S(2,:),S(3,:))              % Trajectory
grid on
axis equal
view(30,20)
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
title('Trajectory along the predetermined shape');


% POSITION, VELOCITY, ACCELERATION IN GRIPPER SPACE
figure('name','Motion along the shape - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);
nexttile
grid on
plot(tt,S(1,:), tt,S(2,:), tt,S(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('gripper position [m]')
legend('x','y','z')
nexttile
grid on
plot(tt,Sp(1,:), tt,Sp(2,:), tt,Sp(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('gripper velocity [m/s]')
legend('x','y','z')
nexttile
grid on
plot(tt,Spp(1,:), tt,Spp(2,:), tt,Spp(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('gripper acceleration [m/s^2]')
legend('x','y','z')

title(t,'Gripper coordinates along the predetermined shape');


% POSITION, VELOCITY, ACCELERATION IN JOINT SPACE
figure('name','Motion along the shape - Joint space','NumberTitle','off')
t = tiledlayout(3,1);
nexttile
grid on
plot(tt,Q(1,:), tt,Q(2,:), tt,Q(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint position')
legend('x [rad]','y [m]','z [rad]')
nexttile
grid on
plot(tt,Qp(1,:), tt,Qp(2,:), tt,Qp(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint velocity')
legend('x [rad/s]','y [m/s]','z [rad/s]')
nexttile
grid on
plot(tt,Qpp(1,:), tt,Qpp(2,:), tt,Qpp(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint acceleration')
legend('x [rad/s^2]','y [m/s^2]','z [rad/s^2]')

title(t,'Joint coordinates along the predetermined shape');


% PLOT POSITION, VELOCITY AND ACCELERATION FOR X, Y AND Z ALONG THE TRAJECTORY
figure('Name','Motion along the shape - Curvilinear abscissa','NumberTitle','off')
plot(tt,pos, tt(1:end-1),vel, tt(1:end-2),acc, [tt(1) tt(end)],[0 0],'k')
grid on
legend('displacement','velocity','acceleration','Location','northwest')
title('Motion along the trajectory')
xlabel('time [s]'), ylabel('[cm] - [cm/s] - [cm/s^2]')

title(t,'Curvilinear absissa, velocity and acceleration along the trajectory');


% PLOT VELOCITY AND ACCELERATION FOR X, Y AND Z  - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion along the shape - Gripper space','NumberTitle','off')
t = tiledlayout(3,1);
nexttile
grid on
plot(tt,Sp(1,:), tt(1:end-1),xp_diff, tt,Spp(1,:), tt(1:end-1),xpp_diff, [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('x direction')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')
nexttile
grid on
plot(tt,Sp(2,:), tt(1:end-1),yp_diff, tt,Spp(2,:), tt(1:end-1),ypp_diff, [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('y direction')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')
nexttile
grid on
plot(tt,Sp(3,:), tt(1:end-1),zp_diff, tt,Spp(3,:), tt(1:end-1),zpp_diff, [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('z direction')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

title(t,{'Gripper coordinates along the predetermined shape', ...
         'Analytical vs numerical solution'});


% PLOT JOINT COORDINATES - ANALYTICAL VS NUMERICAL SOLUTION
figure('name','Motion along the shape - Joint space','NumberTitle','off')
t = tiledlayout(3,1);
nexttile
grid on
plot(tt,Qp(1,:), tt(1:end-1),Qp_diff(1,:), tt,Qpp(1,:), tt(1:end-1),Qpp_diff(1,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint 1')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')
nexttile
grid on
plot(tt,Qp(2,:), tt(1:end-1),Qp_diff(2,:), tt,Qpp(2,:), tt(1:end-1),Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint 2')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')
nexttile
grid on
plot(tt,Qp(3,:), tt(1:end-1),Qp_diff(3,:), tt,Qpp(3,:), tt(1:end-1),Qpp_diff(3,:), [tt(1) tt(end)],[0 0],'k')
xlabel('time [s]'); ylabel('joint 3')
legend('velocity - analytical','velocity - numerical','acceleration - analytical','acceleration - numerical')

title(t,{'Joint coordinates along the predetermined shape', ...
         'Analytical vs numerical solution'});
%}