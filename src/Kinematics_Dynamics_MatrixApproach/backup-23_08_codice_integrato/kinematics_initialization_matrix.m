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
