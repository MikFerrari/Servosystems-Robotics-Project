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