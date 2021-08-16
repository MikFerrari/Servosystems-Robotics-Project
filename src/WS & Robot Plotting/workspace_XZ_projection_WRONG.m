function [] = workspace_XZ_projection_WRONG(L,angle,limits,fig,nPoints)

    q1_inf = limits(1); q1_sup = limits(2);
    q2_inf = limits(3); q2_sup = limits(4);
    q3_inf = limits(5); q3_sup = limits(6);
    
    nPoints_q1 = nPoints(1);
    nPoints_q2 = nPoints(2);
    nPoints_q3 = nPoints(3);
    
    q1 = linspace(q1_inf,q1_sup,nPoints_q1);
    
    figure(fig)
    hold on
    offset = 3;
      
    X3 = []; Y3 = []; Z3 = [];
    X3_LIMINFq1 = []; Y3_LIMINFq1 = []; Z3_LIMINFq1 = [];
    X3_LIMSUPq1 = []; Y3_LIMSUPq1 = []; Z3_LIMSUPq1 = [];
        
    for i = 1:4
        
        if i == 1
            q2 = linspace(q2_sup,q2_sup,1);
            q3 = linspace(q3_inf,q3_sup,nPoints_q3);
        end
        
        if i == 2
            q2 = flip(linspace(q2_inf,q2_sup,nPoints_q2));
            q3 = linspace(q3_sup,q3_sup,1);
        end
        
        if i == 3
            q2 = linspace(q2_inf,q2_inf,1);
            q3 = flip(linspace(q3_inf,q3_sup,nPoints_q3));
        end
        
        if i == 4
            q2 = linspace(q2_inf,q2_sup,nPoints_q2);
            q3 = linspace(q3_inf,q3_inf,1);
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
    
%     plot(X3_LIMINFq1(:),Z3_LIMINFq1(:),'b'); % Plot only one vertical section of the WS
    plot(X3_LIMSUPq1(:),Z3_LIMSUPq1(:),'b');
        
%     vertices = [X3_LIMINFq1; Z3_LIMINFq1]'; % Fill only the area of one vertical section of the WS
%     faces = 1:size(vertices,1);
%     area_color = [0.38 0.69 0.78];
%     patch('Faces',faces,'Vertices',vertices,'FaceColor',area_color,'FaceAlpha',0.25,'EdgeColor','none');
    
    vertices = [X3_LIMSUPq1; Z3_LIMSUPq1]';
    faces = 1:size(vertices,1);
    area_color = [0.38 0.69 0.78];
    patch('Faces',faces,'Vertices',vertices,'FaceColor',area_color,'FaceAlpha',0.25,'EdgeColor','none');
    
    grid on
    axis equal
    xlabel('x'); ylabel('z');

    xlim([min(x3(:))-offset,max(x3(:))+offset])
    ylim([min(z3(:))-offset,max(z3(:))+offset])
    
    title('Projection of the workspace in the XZ plane')
    hold off
    
end

