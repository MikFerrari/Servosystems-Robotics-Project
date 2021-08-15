function [] = workspace_XZ_projection_OLD(L,angle,limits,fig,nPoints)

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
        
    end
    
    P3 = [X3; Y3; Z3];
    idx_planeXZ = abs(P3(2,:))<=0.1;
    P3_planeXZ = P3(:,idx_planeXZ);
    scatter3(P3_planeXZ(1,:),P3_planeXZ(2,:),P3_planeXZ(3,:),1,'b','filled');

    grid on
    axis equal
    view(0,0)
    xlabel('x'); ylabel('y'); zlabel('z')

    xlim([min(x3(:))-offset,max(x3(:))+offset])
    ylim([min(y3(:))-offset,max(y3(:))+offset])
    zlim([min(z0(:))-offset,max(z3(:))+offset])
    
    title('Projection of the workspace in the XZ plane')
    hold off
    
end

