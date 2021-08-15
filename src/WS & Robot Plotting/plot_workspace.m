function [] = plot_workspace(L,angle,limits,fig,nPoints)

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
        
        [q1_grid_LIMq1,q2_grid_LIMq1,q3_grid_LIMq1] = ndgrid([q1_inf q1_sup],q2,q3);
        Q = {q1_grid; q2_grid; q3_grid};
        
        [~,S] = dir_kin(Q,L,angle);
        
        x0 = S{1};    y0 = S{2};    z0 = S{3};
%         x1a = S{4};   y1a = S{5};   z1a = S{6};
%         x1b = S{7};   y1b = S{8};   z1b = S{9};
%         x1c = S{10};  y1c = S{11};  z1c = S{12};
%         x2 = S{13};   y2 = S{14};   z2 = S{15};
        x3 = S{16};   y3 = S{17};   z3 = S{18};
        
        P3 = [x3(:) y3(:) z3(:)];
        P3 = sortrows(P3,3);
        
        P3_split = mat2cell(P3,repelem(size(z3,1), size(P3,1)/size(z3,1)));
        
        cellfun(@(x) plot3(x(:,1),x(:,2),x(:,3),'.-b'),P3_split);

    end
    
    grid on
    axis equal
    view(30,30)
    xlabel('x'); ylabel('y'); zlabel('z')
    xlim([min(x3(:))-offset,max(x3(:))+offset])
    ylim([min(y3(:))-offset,max(y3(:))+offset])
    zlim([min(z0(:))-offset,max(z3(:))+offset])
    title('Robot workspace in the cartesian space')
    hold off
    
end

