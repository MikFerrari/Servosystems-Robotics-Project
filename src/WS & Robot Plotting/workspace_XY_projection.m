function [] = workspace_XY_projection(L,angle,limits,fig,nPoints)

    q1_inf = limits(1); q1_sup = limits(2);
    q2_inf = limits(3); q2_sup = limits(4);
    q3_inf = limits(5); q3_sup = limits(6);
    
    nPoints_q1 = nPoints(1);
    nPoints_q2 = nPoints(2);
    nPoints_q3 = nPoints(3);
    
    q1 = linspace(q1_inf,q1_sup,nPoints_q1);
    
    figure(fig)
    hold on
      
    X3 = []; Y3 = []; Z3 = [];
    X3_LIMINFq1 = []; Y3_LIMINFq1 = []; Z3_LIMINFq1 = [];
    X3_LIMSUPq1 = []; Y3_LIMSUPq1 = []; Z3_LIMSUPq1 = [];
        
    for i = 1:3
        
        if i == 1
            q2 = linspace(q2_sup,q2_sup,1);
            q3 = linspace(q3_inf,q3_sup,nPoints_q3);
        end
        
        if i == 2
            q2 = linspace(q2_inf,q2_inf,1);
            q3 = linspace(q3_inf,q3_sup,nPoints_q3);
        end
        
        if i == 3
            
            q2 = linspace(q2_inf,q2_sup,nPoints_q2);
            q3 = linspace(q3_inf,q3_sup,2);
        end
        
        [q1_grid,q2_grid,q3_grid] = ndgrid(q1,q2,q3);
        Q = {q1_grid; q2_grid; q3_grid};
        
        [~,S] = dir_kin(Q,L,angle);
        
        x0 = S{1};    y0 = S{2};    z0 = S{3};
        x1a = S{4};   y1a = S{5};   z1a = S{6};
        x1b = S{7};   y1b = S{8};   z1b = S{9};
        x1c = S{10};  y1c = S{11};  z1c = S{12};
        x2 = S{13};   y2 = S{14};   z2 = S{15};
        x3 = S{16};   y3 = S{17};   z3 = S{18};

        X3 = [X3 x3(:)'];
        Y3 = [Y3 y3(:)'];
        Z3 = [Z3 z3(:)']; 
        
        % Limits of actuator 1 (base rotation)
        % LOWER LIMIT
        [q1_grid_LIMINFq1,q2_grid_LIMINFq1,q3_grid_LIMINFq1] = ndgrid(q1_inf,q2,q3);
        Q_LIMINFq1 = {q1_grid_LIMINFq1; q2_grid_LIMINFq1; q3_grid_LIMINFq1};
        
        [~,S] = dir_kin(Q_LIMINFq1,L,angle);
        
        x3_LIMINFq1 = S{16};   y3_LIMINFq1 = S{17};   z3_LIMINFq1 = S{18};

        X3_LIMINFq1 = [X3_LIMINFq1 x3_LIMINFq1(:)'];
        Y3_LIMINFq1 = [Y3_LIMINFq1 y3_LIMINFq1(:)'];
        Z3_LIMINFq1 = [Z3_LIMINFq1 z3_LIMINFq1(:)'];
        
        % UPPER LIMIT
        [q1_grid_LIMSUPq1,q2_grid_LIMSUPq1,q3_grid_LIMSUPq1] = ndgrid(q1_sup,q2,q3);
        Q_LIMSUPq1 = {q1_grid_LIMSUPq1; q2_grid_LIMSUPq1; q3_grid_LIMSUPq1};
        
        [~,S] = dir_kin(Q_LIMSUPq1,L,angle);
        
        x3_LIMSUPq1 = S{16};   y3_LIMSUPq1 = S{17};   z3_LIMSUPq1 = S{18};

        X3_LIMSUPq1 = [X3_LIMSUPq1 x3_LIMSUPq1(:)'];
        Y3_LIMSUPq1 = [Y3_LIMSUPq1 y3_LIMSUPq1(:)'];
        Z3_LIMSUPq1 = [Z3_LIMSUPq1 z3_LIMSUPq1(:)']; 
        
    end
    
    if q1_inf ~= -pi && q1_sup ~= pi
        plot(X3_LIMINFq1,Y3_LIMINFq1,'b');
        plot(X3_LIMSUPq1,Y3_LIMSUPq1,'b');
    end
    
    P3 = [X3; Y3; Z3];
    external_limit_idx = vecnorm(P3(1:2,:))>=0.9999*max(vecnorm(P3(1:2,:)));
    external_limit_idx = external_limit_idx & (P3(3,:) == P3(3,find(external_limit_idx,1,'first')));
    external_limit = P3(1:2,external_limit_idx);
    plot(external_limit(1,:),external_limit(2,:),'b');
    
    internal_limit_idx = vecnorm(P3(1:2,:))<1.0001*min(vecnorm(P3(1:2,:)));
    internal_limit_idx = internal_limit_idx & (P3(3,:) == P3(3,find(internal_limit_idx,1,'first')));
    internal_limit = P3(1:2,internal_limit_idx);
    internal_limit = internal_limit(:,1:floor(length(internal_limit)/2));
    plot(internal_limit(1,:),internal_limit(2,:),'b');
    
    vertices = [external_limit(1,:) flip(internal_limit(1,:)) internal_limit(1,1); ...
                external_limit(2,:) flip(internal_limit(2,:)) internal_limit(2,1)]';
    faces = 1:size(vertices,1);
    area_color = [0.38 0.69 0.78];
    patch('Faces',faces,'Vertices',vertices,'FaceColor',area_color,'FaceAlpha',0.25,'EdgeColor','none');

    grid on
    axis equal
    xlabel('x [m]'); ylabel('y [m]');
    
    xlim([1.1*min(X3) 1.1*max(X3)])
    ylim([1.1*min(Y3) 1.1*max(Y3)])
    
    title('Projection of the workspace in the XY plane')
    hold off
    
end

