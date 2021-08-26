%%%%% DYNAMICS INITIALIZATION %%%%%

% POSITION OF THE CENTRES OF MASS
% in the reference frame of the link (following joint) → DISTAL
G1; G2; G3;

% MASSES AND INERTIA MOMENTS
M_links;
J_links;

% GRAVITY ACCELERATION MATRIX
gx = 0; gy = 0; gz = -9.81;
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
%     JJ_end(:,:,j) = M_endTobary*JJ_bary(:,:,j)*M_endTobary';
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