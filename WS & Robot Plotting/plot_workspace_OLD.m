function [] = plot_workspace_OLD(L,angle,limits,fig)
% [] = plot_robot(Q,L,angle,limits,color,fig)
    
    l1a = L(1);
    l1b = L(2);
    l1c = L(3);
    l2 = L(4);
    l3 = L(5);
    
    q1_inf = limits(1); q1_sup = limits(2);
    q2_inf = limits(3); q2_sup = limits(4);
    q3_inf = limits(5); q3_sup = limits(6);
    
    nPoints = 5;
    
    q1 = linspace(q1_inf,q1_sup,10*nPoints);
    
    figure(fig)
    hold on
    offset = 3;
        
    for i = 1:3
        
        if i == 1
            q2 = linspace(q2_sup,q2_sup,nPoints);
            q3 = linspace(q3_inf,q3_sup,5*nPoints);
        end
        
        if i == 2
            q2 = linspace(q2_inf,q2_inf,nPoints);
            q3 = linspace(q3_inf,q3_sup,5*nPoints);
        end
        
        if i == 3
            
            q2 = linspace(q2_inf,q2_sup,5*nPoints);
            q3 = linspace(q3_inf,q3_sup,2);
        end
        
        [q1_grid,q2_grid,q3_grid] = ndgrid(q1,q2,q3);
    
        x0 = 0*ones(size(q1_grid)); y0 = 0*ones(size(q1_grid)); z0 = 0*ones(size(q1_grid));

        x1a = 0*ones(size(q1_grid)); y1a = 0*ones(size(q1_grid)); z1a = l1a*ones(size(q1_grid));

        x1b = l1b*cos(q1_grid); y1b = l1b*sin(q1_grid); z1b = l1a*ones(size(q1_grid));

        x1c = x1b+l1c*cos(angle).*sin(q1_grid); y1c = y1b-l1c*cos(angle).*cos(q1_grid); z1c = z1b+l1c.*sin(angle);

        x2 = x1c+(l2+q2_grid)*cos(angle).*sin(q1_grid); y2 = y1c-(l2+q2_grid)*cos(angle).*cos(q1_grid); z2 = z1c+(l2+q2_grid)*sin(angle);

        x3 = x2+l3*cos(angle+q3_grid).*sin(q1_grid); y3 = y2-l3*cos(angle+q3_grid).*cos(q1_grid); z3 = z2+l3*sin(angle+q3_grid);

        plot3(x0(:),y0(:),z0(:),'o-k')
        plot3(x1a(:),y1a(:),z1a(:),'o-k')
        plot3(x1b(:),y1b(:),z1b(:),'.-r')
        plot3(x1c(:),y1c(:),z1c(:),'.-g')
%         plot3(x2(:),y2(:),z2(:),'.-g')
        plot3(x3(:),y3(:),z3(:),'.-b')
        
    
    end
    
    grid on
    axis equal
    view(30,30)
    xlabel('x'); ylabel('y'); zlabel('z')
    title('Robot workspace in the cartesian space')
    hold off
    xlim([min(x3(:))-offset,max(x3(:))+offset])
    ylim([min(y3(:))-offset,max(y3(:))+offset])
    zlim([min(z0(:))-offset,max(z3(:))+offset])
    
end

