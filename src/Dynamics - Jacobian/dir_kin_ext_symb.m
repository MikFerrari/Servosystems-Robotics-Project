function [S_gripper_symb,S_complete_symb,S_grip_bary] = dir_kin_ext_symb

    syms l1a l1b l1c l2 l3 tilt_angle ...
         x0 y0 z0       x1a y1a z1a     x1b y1b z1b ...
         x1c y1c z1c    x2 y2 z2        x3 y3 z3 ...
         q1_sym(t) q2_sym(t) q3_sym(t) ...
         
    syms G1 [1 3]
    syms G2 [1 3]
    syms G3 [1 3]
    syms q1 q3

    x0 = 0;
    y0 = 0;
    z0 = 0;

    x1a = x0;
    y1a = y0;
    z1a = z0+l1a;

    x1b = x1a+l1b*cos(q1_sym);
    y1b = x1a+l1b*sin(q1_sym);
    z1b = z1a;

    x1c = x1b+l1c*cos(tilt_angle)*sin(q1_sym);
    y1c = y1b-l1c*cos(tilt_angle)*cos(q1_sym);
    z1c = z1b+l1c*sin(tilt_angle);

    x2 = x1c+(l2+q2_sym)*cos(tilt_angle)*sin(q1_sym);
    y2 = y1c-(l2+q2_sym)*cos(tilt_angle)*cos(q1_sym);
    z2 = z1c+(l2+q2_sym)*sin(tilt_angle);

    x3 = x2+l3*cos(tilt_angle+q3_sym)*sin(q1_sym);
    y3 = y2-l3*cos(tilt_angle+q3_sym)*cos(q1_sym);
    z3 = z2+l3*sin(tilt_angle+q3_sym);
    
    % Baricentres
    xg1 = x1b + G1(1);
    yg1 = y1b + G1(2);
    zg1 = z1b + G1(3);
    
    xg2 = x2 + G2(1);
    yg2 = y2 + G2(2);
    zg2 = z2 + G2(3);
    
    xg3 = x3 + G3(1);
    yg3 = y3 + G3(2);
    zg3 = z3 + G3(3);
    
    % Rotation angles
    q1 = q1_sym;
    q3 = q3_sym;
    
    
    S_gripper_symb = [x3; y3; z3];
    S_complete_symb = [x0; y0; z0; x1a; y1a; z1a; x1b; y1b; z1b; x1c; y1c; z1c; x2; y2; z2; x3; y3; z3];
    S_grip_bary = [x3; y3; z3; xg3; yg3; zg3; x2; y2; z2; xg2; yg2; zg2; ...
                   x1c; y1c; z1c; x1b; y1b; z1b; xg1; yg1; zg1; ...
                   x1a; y1a; z1a; q1; q3];
    
end

