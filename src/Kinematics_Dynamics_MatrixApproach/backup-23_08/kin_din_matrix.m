
%% DYNAMICSRRRMANIPULATOR_MATRIX_SIGNFIX
% 3D Dynamics of a 3D RRR manipulator

% The approach adopted consists in defining all matrices and rotations with
% respect to the standard convention (counterclockwise rotations) regardless
% of the given schematic of the manipulator. A code interface converts input
% data from the user into proper values that fit the conventions adopted here.

% clc,close all
%clear


%% Data of the problem

% TARGET MOTION IN JOINT COORDINATES
target_joints_ini = [-0.3*pi; 1; 0.2*pi];       % Initial position in joint coordinates
target_joints_fin = [0.3*pi; 5; -0.3*pi];     % Final position in joint coordinates

% % FIX THE SIGNS ACCORDING TO THE GIVEN CONVENTION 
% signFix = [1; 1; -1];
% Qini = signFix.*target_angle_ini;
% Qfin = signFix.*target_angle_fin;
Qini = target_joints_ini;
Qfin = target_joints_fin;

% GEOMETRICAL PROPERTIES
nLinks = 4;
% h = 0.2; l1 = 0.8; l2 = 0.7; l3 = 0.4;
% L = [h l1 l2 l3];

%{
% POSITION OF THE CENTRES OF MASS
% Distance from PROXIMAL joint
g0 = nan; g1 = 0.48; g2 = 0.42; g3 = 0.24;
Gpos = [g0 g1 g2 g3];

% MASSES AND INERTIA MOMENTS
m0 = nan; m1 = 8; m2 = 7; m3 = 4;
M_links = [m0 m1 m2 m3];

Jg0 = nan; Jg1 = 0.427; Jg2 = 0.286; Jg3 = 0.0533;
Jg_links = [Jg0 Jg1 Jg2 Jg3];

% EXTERNAL FORCES AND TORQUES ON THE GRIPPER
% forceValue = 0;
forceValue = 1;
fx = forceValue; fy = forceValue; fz = forceValue;
F = [fx; fy; fz];

torqueValue = 0;
cx = torqueValue; cy = torqueValue; cz = torqueValue;
C = [cx; cy; cz];
%}

% LAW OF MOTION DATA
Ttot = 5;                 % total time of motion = 5 seconds
nPoints = 100;            % number of points
dT = Ttot/(nPoints-1);    % time step
dq1 = Qfin(1)-Qini(1);    % alpha
dq2 = Qfin(2)-Qini(2);    % beta
dq3 = Qfin(3)-Qini(3);    % gamma


%% Plot initial and final configuration of the manipulator

% figure('name','3D representation xyz','NumberTitle','off')
% p1 = plotRRR(target_angle_ini,L,'r',gcf);	% Initial configuration
% hold on
% p2 = plotRRR(target_angle_fin,L,'b',gcf);	% Final configuration
% grid on
% title('RRR 3D manipulator initial and final poses');
% legend([p1(1) p2(1)],'initial pose','final pose');
% xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
% view(27,26) % Point of view: (azimut,elevation)


%% Second Approach: 3D Kinematics + Dynamics using 4x4 Homogeneus Matrices

%%%%% NAMING CONVENTION %%%%%
% Xab_c ??? matrix of b wrt a in reference frame c
% XX_c ??? array of X matrices in reference frame c
% M ??? position matrix of a reference frame
% W ??? velocity matrix of a reference frame
% H ??? acceleration matrix of a reference frame
% Mg ??? position matrix of a reference frame put in the center of mass of a link
% GRIPPER SPACE ??? x-y-z
% JOINT SPACE ??? q1-q2-q3
%%%%% END NAMING CONVENTION %%%%%


%%%%% KINEMATICS INITIALIZATION %%%%%

% DESCRIBE DEGREES OF FREEDOM OF THE JOINTS
L01_0 = [0 -1 0 0;
         1  0 0 0;
         0  0 0 0;
         0  0 0 0]; % Lz
   
L12_1 = [0 0 0 0;
         0 0 0 0;
         0 0 0 1;
         0 0 0 0]; % Lz
   
L23_2 = [0 -1 0 0;
         1  0 0 0;
         0  0 0 0;
         0  0 0 0]; % Lz

LL_prev = [L01_0 L12_1 L23_2];
LL_prev = reshape(LL_prev,4,4,nLinks-1);  % L matrices wrt the previous joint
LL_0 = zeros(4,4,nLinks-1);               % L matrices wrt the base frame

% BASE LINK MATRICES
M00_0 = eye(4); W00_0 = zeros(4); H00_0 = zeros(4); Mg00_prev = nan(4);

% ARRAYS OF POSITION, VELOCITY, ACCELERATION WRT THE BASE FRAME
MM_0 = NaN(4,4,nLinks,nPoints);
WW_0 = NaN(4,4,nLinks,nPoints);
HH_0 = NaN(4,4,nLinks,nPoints);

% ARRAYS OF RELATIVE VELOCITY WRT THE BASE FRAME
WW_rel_0 = zeros(4,4,nLinks,nPoints);

% ARRAY OF POSITION MATRICES OF THE CENTER OF MASS WRT THE PREVIOUS FRAME
MMg_prev = zeros(4,4,nLinks,nPoints);

% ARRAY OF POSITION, VELOCITY, ACCELERATION OF THE JOINTS IN THE JOINT SPACE
Q = zeros(nLinks-1,nPoints);
Qp = zeros(nLinks-1,nPoints);
Qpp = zeros(nLinks-1,nPoints);
    
% ARRAY OF POSITION, VELOCITY, ACCELERATION OF THE JOINTS IN THE GRIPPER SPACE
SS = zeros(3,nLinks,nPoints);
SSp = zeros(3,nLinks,nPoints);
SSpp = zeros(3,nLinks,nPoints);

% ARRAY OF POSITION, VELOCITY, ACCELERATION OF THE JOINTS IN THE GRIPPER SPACE ALONG THE TRAJECTORY
PP = zeros(1,nLinks,nPoints);
VV = zeros(1,nLinks,nPoints);
AA = zeros(1,nLinks,nPoints);
        
%%%%% END KINEMATICS INITIALIZATION %%%%%


%%%%% DYNAMICS INITIALIZATION %%%%%
%{
% GRAVITY ACCELERATION MATRIX
gx = 0; gy = 0;
gz = -9.81; % Vertical workspace
% gz = 0;  % Horizontal workspace
Hg = zeros(4,4); Hg(1:3,4) = [gx; gy; gz];
  
% INERTIA MATRICES
JJtilde_bary = zeros(3,3,nLinks);
JJ_bary = zeros(4,4,nLinks);        % J matrices wrt the center of mass of the current link
JJ_end = zeros(4,4,nLinks);         % J matrices wrt the end of the current link
JJ_0 = zeros(4,4,nLinks,nPoints);   % J matrices wrt the base frame

for j = 1:nLinks 
    % Inertia moments wrt baricentral frame of the link
    Jx = 0; Jy = Jg_links(j); Jz = Jg_links(j);
    Ixx = (-Jx+Jy+Jz)/2; Iyy = (Jx-Jy+Jz)/2; Izz = (Jx+Jy-Jz)/2;

    Jxy = 0; Jyz = 0; Jzx = 0;
    Ixy = -Jxy; Iyz = -Jyz; Ixz = -Jzx;
    
    % Define Pseudo-Inertia and Mass matrix
    JJtilde_bary(:,:,j) = [Ixx Ixy Ixz;
                           Ixy Iyy Iyz;
                           Ixz Iyz Izz];
    
    JJ_bary(:,:,j) = [JJtilde_bary(:,:,j)     [0;0;0];
                      [0;0;0]'             M_links(j)];              
     
    M_endTobary = eye(4);
    M_endTobary(1,4) = -(L(j)-Gpos(j));
    
    % Convert Pseudo-Inertia and Mass matrix to the frame at the end of the link    
    JJ_end(:,:,j) = M_endTobary*JJ_bary(:,:,j)*M_endTobary';
end

% EXTERNAL FORCE AND TORQUE MATRIX
c_ = [  0 -cz  cy;
       cz   0 -cx;
      -cy  cx   0]; % Matrix representation of torque vector
Phi_ext_a = [c_  F;
             -F' 0]; % Expressed in an auxiliary frame,
                     % located at the end of the last link and whose axes
                     % are parallel to the ones of the base frame

% ARRAYS OF FORCE-TORQUE MATRICES WRT THE BASE FRAME
Phi_g_array_0 = zeros(4,4,nLinks,nPoints);
Phi_in_array_0 = zeros(4,4,nLinks,nPoints);
Phi_vinc_array_0 = zeros(4,4,nLinks,nPoints);

% ARRAY OF FORCES-TORQUES OF THE ACTUATORS WRT THE BASE FRAME
phi_actuators_array_0 = zeros(1,1,nLinks,nPoints);

% ENERGY AND POWER OF EACH LINK WRT THE BASE FRAME
Ek = zeros(nLinks,nPoints);
Ep = zeros(nLinks,nPoints);
E = zeros(nLinks,nPoints);
Wext = zeros(1,nPoints);
Wq = zeros(nLinks,nPoints);
W = zeros(1,nPoints);
Wweight_link = zeros(nLinks,nPoints);   % power of the weight force for each link
Wweight = zeros(1,nPoints);             % sum of power of the weight forces for all links

%%%%% END DYNAMICS INITIALIZATION %%%%%
%}

%%%%% LOOP FOR EACH TIME STEP %%%%%

tt = linspace(0,Ttot,nPoints);

for i = 1:nPoints
    
    %%%%% KINEMATICS (from the base link to the end-effector) %%%%%
    
    % JOINT COORDINATES
    % Cycloidal law of motion
    [q1,q1p,q1pp] = cycloidal(tt(i),Ttot,Qini(1),dq1);
    [q2,q2p,q2pp] = cycloidal(tt(i),Ttot,Qini(2),dq2);
    [q3,q3p,q3pp] = cycloidal(tt(i),Ttot,Qini(3),dq3);
    
    % Save joint coordinates
    Q(:,i) = [q1; q2; q3];
    Qp(:,i) = [q1p; q2p; q3p];
    Qpp(:,i) = [q1pp; q2pp; q3pp];
    
    
    % LOOP THROUGH EACH LINK TO COMPUTE POSITION, VELOCITY, ACCELERATION MATRICES
    
    % BASE LINK
    MM_0(:,:,1,i) = M00_0;
    WW_0(:,:,1,i) = W00_0;
    HH_0(:,:,1,i) = H00_0;
    MMg_prev(:,:,1,i) = Mg00_prev;
    
    
    for j = 2:nLinks
        %{
        switch j
            case 2
                % FIRST LINK
                % Frame rotation wrt the previous frame
                R = [cos(q1) -sin(q1) 0;
                     sin(q1)  cos(q1) 0;
                         0           0      1];
                % Distal end of the link     
                T = [l1*cos(q1); l1*sin(q1); h];
                
                % Center of mass
%                 Tg = [g1*cos(q1); g1*sin(q1); h];
                
            case 3
                % SECOND LINK
                % Frame rotation wrt the previous frame
                R = [cos(q2) -sin(q2) 0;
                     sin(q2)  cos(q2) 0;
                         0           0      1];
                     
                % Distal end of the link     
                T = [l2*cos(q2); l2*sin(q2); 0];
                
                % Center of mass
%                 Tg = [g2*cos(q2); g2*sin(q2); 0];
                
            case 4
                % THIRD LINK
                % Frame rotation wrt the previous frame
                R = [ cos(q3)    0     sin(q3);
                         0       1        0   ;
                     -sin(q3)    0     cos(q3)];
                 
                % Distal end of the link  
                T = [l3*cos(q3); 0; l3*(-sin(q3))];
                
                % Center of mass
%                 Tg = [g3*cos(q3); 0; g3*(-sin(q3))];   
                
        end
        %}

        % POSITION OF REFERENCE FRAME
        % Position matrix of the current frame wrt the PREVIOUS frame
        M_rel_prevRef = Mall{j-1}(q1,q2,q3);
%         Mg_rel_prevRef = [   R    Tg;
%                           [0 0 0]  1];

        % Position matrix of the current frame wrt the BASE frame
        M_abs_baseRef = MM_0(:,:,j-1,i)*M_rel_prevRef;
        % Save new matrix into position matrices array
        MM_0(:,:,j,i) = M_abs_baseRef;
        
        % Position matrix of the CENTER OF MASS wrt the previous frame
%         Mg_rel_prevRef = [   R    Tg;
%                           [0 0 0]  1];
        % Save new matrix into position matrices array
%         MMg_prev(:,:,j,i) = Mg_rel_prevRef;
        
        % VELOCITY OF REFERENCE FRAME
        % Current frame wrt the PREVIOUS frame expressed in the PREVIOUS frame
        W_rel = LL_prev(:,:,j-1).*Qp(j-1,i);          
        % Current frame wrt the PREVIOUS frame expressed in the BASE frame
        W_rel_0 = MM_0(:,:,j-1,i)*W_rel*MM_0(:,:,j-1,i)^(-1);
        % Current frame wrt the BASE frame expressed in the BASE frame (RIVALS' THEOREM)
        W_abs_baseRef = WW_0(:,:,j-1,i)+W_rel_0;      
        % Save new matrix into velocity matrices array
        WW_0(:,:,j,i) = W_abs_baseRef;                            
        WW_rel_0(:,:,j,i) = W_rel_0;
                
        % ACCELERATION OF REFERENCE FRAME
        % Current frame wrt the PREVIOUS frame expressed in the PREVIOUS frame
        H_rel = LL_prev(:,:,j-1)*Qpp(j-1,i)+LL_prev(:,:,j-1)^2*Qp(j-1,i)^2;
        % Current frame wrt the PREVIOUS frame expressed in the BASE frame
        H_rel_0 = MM_0(:,:,j-1,i)*H_rel*MM_0(:,:,j-1,i)^(-1);
        % Current frame wrt the BASE frame expressed in the BASE frame (CORIOLIS' THEOREM)
        H_abs_baseRef = HH_0(:,:,j-1,i)+H_rel_0+2*WW_0(:,:,j-1,i)*W_rel_0;
        % Save new matrix into acceleration matrices array
        HH_0(:,:,j,i) = H_abs_baseRef;
        
        % JOINT ABSOLUTE XYZ POSITION, VELOCITY, ACCELERATION
        SS(:,j,i) = MM_0(1:end-1,4,j,i);
        SSp(:,j,i) = WW_0(1:end-1,:,j,i)*MM_0(:,4,j,i);
        SSpp(:,j,i) = HH_0(1:end-1,:,j,i)*MM_0(:,4,j,i);
        
        % JOINT ABSOLUTE POSITION, VELOCITY, ACCELERATION ALONG THE TRAJECTORY
        PP(:,j,i) = norm(SS(:,j,i) - SS(:,j,1));
        if i == 1
            VV(:,j,i) = norm(SSp(:,j,i));
            AA(:,j,i) = norm(SSpp(:,j,i));
        elseif i > 1
            if PP(:,j,i) >= PP(:,j,i-1)
                VV(:,j,i) = norm(SSp(:,j,i));     end
            if PP(:,j,i) < PP(:,j,i-1)
                VV(:,j,i) = -norm(SSp(:,j,i));    end
            if VV(:,j,i) >= VV(:,j,i-1)
                AA(:,j,i) = norm(SSpp(:,j,i));    end
            if VV(:,j,i) < VV(:,j,i-1)
                AA(:,j,i) = -norm(SSpp(:,j,i));   end
        end
    
    end
    
    %%%%% END KINEMATICS %%%%%
    
    %{
    %%%%% DYNAMICS (from the end-effector to the base link) %%%%%      
    
    % CONVERT EXTERNAL FORCES-TORQUES ON THE GRIPPER TO THE BASE REFERENCE FRAME
    % External forces and torques are defined in an auxiliary frame
    M0a_0 = eye(4);
    M0a_0(1:3,4) = MM_0(1:3,4,4,i);
    Phi_ext_0 = M0a_0*Phi_ext_a*M0a_0';
    
    % VELOCITY MATRIX OF THE AUXILIARY FRAME WRT THE BASE FRAME
    W0a_0 = zeros(4);
    W0a_0(1:3,4) = SSp(:,4,i);
        
    for j = nLinks:(-1):1
        
        % CONVERT L MATRICES TO THE BASE REFERENCE FRAME
        if j > 1
            LL_0(:,:,j-1) = MM_0(:,:,j-1,i)*LL_prev(:,:,j-1)*MM_0(:,:,j-1,i)^(-1);
        end
        
        % CONVERT J MATRICES TO THE REFERENCE FRAME
        JJ_0(:,:,j,i) = MM_0(:,:,j,i)*JJ_end(:,:,j)*MM_0(:,:,j,i)';

        % BODY WEIGHT IN THE BASE REFERENCE FRAME (no need to convert)
        Phi_g = skew(Hg*JJ_0(:,:,j,i));
        Phi_g_array_0(:,:,j,i) = Phi_g;
        
        % INERTIAL FORCES AND TORQUES IN THE BASE REFERENCE FRAME (no need to convert)
        Phi_in = skew(HH_0(:,:,j,i)*JJ_0(:,:,j,i));
        Phi_in_array_0(:,:,j,i) = Phi_in;
        
        % VINCULAR REACTION TRANSMITTED FROM THE CURRENT LINK TO THE PREVIOUS ONE,
        % EXPRESSED IN THE BASE REFERENCE FRAME
        if j == nLinks
            Phi_vinc_array_0(:,:,j,i) = -Phi_in_array_0(:,:,j,i)+Phi_g_array_0(:,:,j,i)+Phi_ext_0;
        else
            Phi_vinc_array_0(:,:,j,i) = -Phi_in_array_0(:,:,j,i)+Phi_g_array_0(:,:,j,i)+Phi_vinc_array_0(:,:,j+1,i);
        end
        
        % ACTUATOR FORCES AND TORQUES IN THE JOINT BETWEEN THE CURRENT LINK AND THE PREVIOUS ONE
        if j > 1
            phi_actuators_array_0(1,1,j,i) = -pseudoScalar(Phi_vinc_array_0(:,:,j,i),signFix(j-1)*LL_0(:,:,j-1));
        end
        
        % KINETIC & POTENTIAL ENERGY OF THE CURRENT LINK WRT THE BASE FRAME
        Ek(j,i) = trace(0.5*WW_0(:,:,j,i)*JJ_0(:,:,j,i)*WW_0(:,:,j,i)');
        Ep(j,i) = -trace(Hg*JJ_0(:,:,j,i));
        E(j,i) = Ek(j,i)+Ep(j,i);   % total mechanical energy
        
        % MOTOR POWER IN THE CURRENT JOINT (frame-independent?)
        if j > 1
            Wq(j,i) = -pseudoScalar(Phi_vinc_array_0(:,:,j,i),WW_rel_0(:,:,j,i));
            % "-" because "Phi_vinc_array_0" applied to the link is the
            % opposite to "Phi_vinc_array_0" transmitted to the previous one
            
            Wweight_link(j,i) = pseudoScalar(Phi_g_array_0(:,:,j,i),WW_0(:,:,j,i));
            % "+" because "Phi_g_array_0" is a force that acts in the
            % absolute frame and "WW_0" are absolute velocity matrices
            
%             Wweight_link(j,i) = trace(WW_0(:,:,j,i)*JJ_0(:,:,j,i)*Hg'); % Alternative method to compute power
        end
        
    end
    
    % EXTERNAL FORCES-TORQUES POWER (frame-independent?)
    Wext(i) = pseudoScalar(Phi_ext_0,W0a_0);  
    % "+" because "Phi_ext_0" is a force that acts in the
    % absolute frame and "W0a_0" is an absolute velocity matrix
            
    % TOTAL POWER (from motors and external forces/torques)
    W(i) = sum(Wq(:,i))+Wext(i);	
    
    % POWER OF THE WEIGHT FORCES
    Wweight(i) = sum(Wweight_link(:,i));
    
    %%%%% END DYNAMICS %%%%%
%}      
end 
  

%%%%% END LOOP FOR EACH TIME STEP %%%%%


%{
% TOTAL ENERGY OF THE WHOLE BODY (sum of the energy of each link)
Ek_tot = sum(Ek,1,'omitnan');
Ep_tot = sum(Ep,1,'omitnan');
E_tot = sum(E,1,'omitnan');

% TOTAL POWER OF THE WHOLE BODY (sum of the power of each link) and
% POWER OF JUST WEIGHT FORCES
Wq_tot = sum(Wq,1,'omitnan');
Wext_tot = Wext;
W_tot = sum(W,1,'omitnan');
W_tot_plus_weight = W_tot+sum(Wweight,1,'omitnan');
%}


%% Compute derivatives (numerical differentiation) to debug results

% DERIVATIVE OF TOTAL ENERGY and JUST KINETIC ENERGY
% E_tot_diff = diff(E_tot)/dT;
% Ek_tot_diff = diff(Ek_tot)/dT;

% VELOCITY AND ACCELERATION - GRIPPER COORDINATES
SSp_diff = zeros(3,nLinks,nPoints-1);
SSpp_diff = zeros(3,nLinks,nPoints-1);
VV_diff = zeros(1,nLinks,nPoints-1);
AA_diff = zeros(1,nLinks,nPoints-1);

for j = 2:nLinks
    S = reshape(SS(:,j,:),3,[]);
    SSp_diff(:,j,:) = diff(S,[],2)/dT;
    
    Sp = reshape(SSp(:,j,:),3,[]);
    SSpp_diff(:,j,:) = diff(Sp,[],2)/dT;
    
    for i = 2:nPoints-1
        if i == 1
            VV_diff(:,j,i) = norm(SSp(:,j,i));
            AA_diff(:,j,i) = norm(SSpp(:,j,i));
        elseif i > 1
            if PP(:,j,i) >= PP(:,j,i-1)
                VV_diff(:,j,i) = norm(SSp(:,j,i));      end
            if PP(:,j,i) < PP(:,j,i-1)
                VV_diff(:,j,i) = -norm(SSp(:,j,i));     end
            if VV_diff(:,j,i) >= VV_diff(:,j,i-1)
                AA_diff(:,j,i) = norm(SSpp(:,j,i));     end
            if VV_diff(:,j,i) < VV_diff(:,j,i-1)
                AA_diff(:,j,i) = -norm(SSpp(:,j,i));    end
        end
    end
end

% VELOCITY AND ACCELERATION - JOINT COORDINATES
Qp_diff = diff(Q,[],2)/dT;
Qpp_diff = diff(Qp,[],2)/dT;


%% Plot kinematics results

% RENAME VARIABLES TO PLOT MORE EASILY
% Gripper position
x_grip = reshape(SS(1,end,:),1,[]);
y_grip = reshape(SS(2,end,:),1,[]);
z_grip = reshape(SS(3,end,:),1,[]);
% Gripper velocity
xp_grip = reshape(SSp(1,end,:),1,[]);   xp_diff_grip = reshape(SSp_diff(1,end,:),1,[]);
yp_grip = reshape(SSp(2,end,:),1,[]);   yp_diff_grip = reshape(SSp_diff(2,end,:),1,[]);
zp_grip = reshape(SSp(3,end,:),1,[]);   zp_diff_grip = reshape(SSp_diff(3,end,:),1,[]);
% Gripper acceleration
xpp_grip = reshape(SSpp(1,end,:),1,[]);   xpp_diff_grip = reshape(SSpp_diff(1,end,:),1,[]);
ypp_grip = reshape(SSpp(2,end,:),1,[]);   ypp_diff_grip = reshape(SSpp_diff(2,end,:),1,[]);
zpp_grip = reshape(SSpp(3,end,:),1,[]);   zpp_diff_grip = reshape(SSpp_diff(3,end,:),1,[]);
% Gripper motion along the trajectory
pos_grip = reshape(PP(1,end,:),1,[]);
vel_grip = reshape(VV(1,end,:),1,[]);   vel_diff_grip = reshape(VV_diff(1,end,:),1,[]);
acc_grip = reshape(AA(1,end,:),1,[]);   acc_diff_grip = reshape(AA_diff(1,end,:),1,[]);


% INITIAL & FINAL CONFIGURATION + TRAJECTORY
figure('name','3D representation xyz + trajectory','NumberTitle','off')
p1 = DH_plot_robot(target_joints_ini',D,A,alpha,gcf,'r',0);    % Initial configuration
view(34,38)
hold on
p2 = DH_plot_robot(target_joints_fin',D,A,alpha,gcf,'b',0);    % Final configuration

for j = 1:nLinks
    hold on
    p3 = plot3(reshape(SS(1,j,:),1,[]),reshape(SS(2,j,:),1,[]),reshape(SS(3,j,:),1,[]),'g');
end

grid on
title('RRR 3D manipulator poses and joint trajectory');
legend([p1(1) p2(1) p3],'initial pose','final pose','joint trajectory');
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
view(27,26) % Point of view: (azimut,elevation)

%%
% GRIPPER POSITION, VELOCITY, ACCELERATION - ONLY ANALYTICAL SOLUTION
figure('Name','Motion along x','NumberTitle','off')
plot(tt,x_grip,  tt,xp_grip, tt,xpp_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along x-axis')
legend('displacement','velocity','acceleration','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along y','NumberTitle','off')
plot(tt,y_grip,  tt,yp_grip, tt,ypp_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along y-axis')
legend('displacement','velocity','acceleration','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along z','NumberTitle','off')
plot(tt,z_grip,  tt,zp_grip, tt,zpp_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along z-axis')
legend('displacement','velocity','acceleration','Location','northeast')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along the trajectory','NumberTitle','off')
plot(tt,pos_grip,  tt,vel_grip, tt,acc_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along the trajectory')
legend('displacement','velocity','acceleration','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')


% % JOINT POSITION, VELOCITY, ACCELERATION IN JOINT SPACE - ONLY ANALYTICAL SOLUTION
% figure('Name','Motion in the joint space','NumberTitle','off')
% plot(tt,Q(1,:), tt,Q(2,:))
% hold on
% plot(tt,Qp(1,:), tt,Qp(2,:), 'Linewidth',1.5)
% plot(tt,Qpp(1,:), tt,Qpp(2,:), [tt(1) tt(end)],[0 0],'k')
% title('Motion in the joint space')
% legend('$\alpha$','$\beta$','$\stackrel{.}{\alpha}$','$\stackrel{.}{\beta}$', ...
%        '$\stackrel{..}{\alpha}$','$\stackrel{..}{\beta}$', ...
%        'interpreter','latex','Location','southeast')
% xlabel('time'), ylabel('degrees')


% GRIPPER POSITION, VELOCITY, ACCELERATION - ANALYTICAL VS NUMERICAL SOLUTION
figure('Name','Motion along x','NumberTitle','off')
plot(tt,xp_grip, tt(1:end-1),xp_diff_grip, tt,xpp_grip, tt(1:end-1),xpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along x-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along y','NumberTitle','off')
plot(tt,yp_grip, tt(1:end-1),yp_diff_grip, tt,ypp_grip, tt(1:end-1),ypp_diff_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along y-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along z','NumberTitle','off')
plot(tt,zp_grip, tt(1:end-1),zp_diff_grip, tt,zpp_grip, tt(1:end-1),zpp_diff_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along z-axis')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','northeast')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')

figure('Name','Motion along the trajectory','NumberTitle','off')
plot(tt,vel_grip, tt(1:end-1),vel_diff_grip, tt,acc_grip, tt(1:end-1),acc_diff_grip, [tt(1) tt(end)],[0 0],'k')
grid on
title('Motion along the trajectory')
legend('velocity','velocity - diff','acceleration','acceleration - diff','Location','southwest')
xlabel('time'), ylabel('cm - cm/s - cm/s^2')


% % JOINT POSITION, VELOCITY, ACCELERATION IN JOINT SPACE - ANALYTICAL VS NUMERICAL SOLUTION
% figure('Name','Motion in the joint space','NumberTitle','off')
% plot(tt,Qp(1,:), tt(1:end-1),Qp_diff(1,:), tt,Qp(2,:), tt(1:end-1),Qp_diff(2,:),'Linewidth',1.5)
% hold on
% plot(tt,Qpp(1,:), tt(1:end-1),Qpp_diff(1,:), tt,Qpp(2,:), tt(1:end-1),Qpp_diff(2,:), [tt(1) tt(end)],[0 0],'k')
% title('Motion in the joint space')
% legend('$\stackrel{.}{\alpha}$','$\stackrel{.}{\alpha}$ - diff', ...
%        '$\stackrel{.}{\beta}$','$\stackrel{.}{\beta}$ - diff', ...
%        '$\stackrel{..}{\alpha}$','$\stackrel{..}{\alpha}$ - diff', ...
%        '$\stackrel{..}{\beta}$','$\stackrel{..}{\beta}$ - diff', ...
%        'interpreter','latex','Location','southeast')
% xlabel('time'), ylabel('degrees')



%% Plot dynamics results

%{
% FORCES/TORQUES APPLIED BY THE ACTUATORS
figure('Name',strcat("Actuator torques [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Actuator torques - [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
actuatorTorques = reshape(phi_actuators_array_0,4,1,nPoints,[]);
titles = {'','Torque joint 1','Torque joint 2','Torque joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorTorques(i,:), [tt(1) tt(end)],[0 0],'k')
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% POWER DEVELOPED BY THE ACTUATORS
figure('Name',strcat("Actuator power [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Actuator power [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
actuatorPower = reshape(Wq,4,1,nPoints,[]);
titles = {'','Power joint 1','Power joint 2','Power joint 3'};
for i = 2:4
    nexttile
    plot(tt, actuatorPower(i,:), [tt(1) tt(end)],[0 0],'k')
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% KINETIC, POTENTIAL & TOTAL ENERGY
figure('Name',strcat("Kinetic and potential energy - [F_x = F_y = F_z = ",num2str(forceValue)," N]"),'NumberTitle','off')
t = tiledlayout(3,1,'TileSpacing','none');
title(t,strcat("Kinetic and potential energy - [F_x = F_y = F_z = ",num2str(forceValue)," N]"));
titles = {'Potential Energy','Kinetic Energy','Total Energy (E_k+E_p)'};
EE = [Ep_tot; Ek_tot; E_tot];
for i = 1:3
    nexttile
    plot(tt, EE(i,:))
    grid on
    title(titles{i})
    xlabel('time'), ylabel('Nm')
end

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (1st method)
figure('Name','Power equality','NumberTitle','off')
plot(tt(1:end-1),E_tot_diff, tt,W_tot, [tt(1) tt(end)],[0 0],'k')
grid on
title('Derivative of total energy vs power of actuators and external forces')
legend('dE_{tot}/dt','W_{q}+W_{ext}','Location','southwest')
xlabel('time [s]'), ylabel('power [W]')

% MECHANICAL ENERGY CONSERVATION TO CHECK DYNAMICAL ANALYSIS (2nd method)
figure('Name','Power equality','NumberTitle','off')
plot(tt(1:end-1),Ek_tot_diff, tt,W_tot_plus_weight, [tt(1) tt(end)],[0 0],'k')
grid on
title('Derivative of kinetic energy vs power of actuators, external and weight forces')
legend('dE_{k}/dt','W_{q}+W_{ext}+W_{weight}','Location','southwest')
xlabel('time [s]'), ylabel('power [W]')
%}