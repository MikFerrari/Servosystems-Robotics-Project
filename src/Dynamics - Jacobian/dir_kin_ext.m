function [S_gripper,S_complete,S_grip_bary] = dir_kin_ext(Q,L,angle,Gpos)

    l1a = L(1); l1b = L(2); l1c = L(3); l2 = L(4); l3 = L(5);

    x0 = 0*ones(size(Q{1}));
    y0 = 0*ones(size(Q{1}));
    z0 = 0*ones(size(Q{1}));

    x1a = x0+0*ones(size(Q{1}));
    y1a = y0+0*ones(size(Q{1}));
    z1a = z0+l1a*ones(size(Q{1}));

    x1b = x1a+l1b*cos(Q{1});
    y1b = x1a+l1b*sin(Q{1});
    z1b = z1a;

    x1c = x1b+l1c*cos(angle).*sin(Q{1});
    y1c = y1b-l1c*cos(angle).*cos(Q{1});
    z1c = z1b+l1c.*sin(angle);

    x2 = x1c+(l2+Q{2})*cos(angle).*sin(Q{1});
    y2 = y1c-(l2+Q{2})*cos(angle).*cos(Q{1});
    z2 = z1c+(l2+Q{2})*sin(angle);

    x3 = x2+l3*cos(angle+Q{3}).*sin(Q{1});
    y3 = y2-l3*cos(angle+Q{3}).*cos(Q{1});
    z3 = z2+l3*sin(angle+Q{3});

    % Baricentres
    xg1 = x1b + Gpos{2}(1);
    yg1 = y1b + Gpos{2}(2);
    zg1 = z1b + Gpos{2}(3);

    xg2 = x2 + Gpos{3}(1);
    yg2 = y2 + Gpos{3}(2);
    zg2 = z2 + Gpos{3}(3);

    xg3 = x3 + Gpos{4}(1);
    yg3 = y3 + Gpos{4}(2);
    zg3 = z3 + Gpos{4}(3);

    % Rotation angles
    q1 = Q{1};
    q3 = Q{3};


    S_gripper = {x3; y3; z3};
    S_complete = {x0; y0; z0; x1a; y1a; z1a; x1b; y1b; z1b; x1c; y1c; z1c; x2; y2; z2; x3; y3; z3};
    S_grip_bary = {x3; y3; z3; xg3; yg3; zg3; x2; y2; z2; xg2; yg2; zg2; ...
                   x1c; y1c; z1c; x1b; y1b; z1b; xg1; yg1; zg1; ...
                   x1a; y1a; z1a; q1; q3};
    
end

