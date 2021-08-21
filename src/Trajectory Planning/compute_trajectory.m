function spline_points_target = compute_trajectory(nPoints_spline,dilation_factor, ...
                                                   x_tilt,y_tilt,z_tilt, ...
                                                   x_offset,y_offset,z_offset)

    %% 2D trajectory sampling
    P = [[0.90; 0.07; 0] [0.80; 0.04; 0] [0.63; 0.02; 0] [0.55; 0.03; 0] [0.44; 0.08; 0] ...
         [0.43; 0.18; 0] [0.56; 0.33; 0] [0.75; 0.52; 0] [0.75; 0.73; 0] [0.58; 0.92; 0] ...
         [0.38; 0.98; 0] [0.19; 0.94; 0] [0.06; 0.79; 0] [0.16; 0.58; 0] [0.34; 0.45; 0] ...
         [0.49; 0.40; 0] [0.61; 0.38; 0]]; % Points along the trajectory

    width_original_image = 127;
    height_original_image = 225.6;
    ratio = height_original_image/width_original_image;

    P(2,:) = P(2,:)*ratio;

    %% Curvilinear abscissa
    dist = zeros(1,size(P,2));
    for i = 2:size(P,2)
        dist(i) = dist(i-1) + norm(P(:,i)-P(:,1));
    end

    %% Spline interpolation

    % N.B.: Specify the second input with two extra values [0 y 0] to signify
    % that the endpoint slopes are both zero! Velocity must be zero at the
    % beginning and at the end of the trajectory
    piecewise_poly = spline(dist(1,:),[[0;0] P(1:2,:) [0;0]]);

    spline_points = ppval(piecewise_poly,linspace(0,dist(end),nPoints_spline));
    spline_points = [spline_points; zeros(1,nPoints_spline)];

    %% Scale the whole shape (compression or dilation)
    spline_points = dilation_factor*spline_points;

    %% Shape in the XYZ space
    % The trajectory must lie all in the robot workspace
    % (possibly away from the singular configurations).

    % Specify target plane (rototranslation of the shape)
    Rx = [1     0            0
          0 cos(x_tilt) -sin(x_tilt)
          0 sin(x_tilt)  cos(x_tilt)];

    Ry = [ cos(y_tilt) 0 sin(y_tilt)
               0       1     0
          -sin(y_tilt) 0 cos(y_tilt)];

    Rz = [cos(z_tilt) -sin(z_tilt) 0
          sin(z_tilt)  cos(z_tilt) 0
              0            0       1];

    R = Rx*Ry*Rz;

    T = [x_offset; y_offset; z_offset];

    M_target = [  R   T
                0 0 0 1];

    spline_points_homo = [spline_points; ones(1,size(spline_points,2))]; 
    spline_points_target = M_target*spline_points_homo;

end