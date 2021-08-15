function [] = DH_plot_robot(Q,D,A,alpha,fig,DHdirect)

    S = DHdirect(Q(1),Q(2),Q(3));
    
    x0 = 0;         y0 = 0;                         z0 = 0;
    x1a = x0;       y1a = y0;                       z1a = D(1);
    x1b = A(1);     y1b = y1a;                      z1b = z1a;
    x2 = x1b;       y2 = -(D(2)+Q(2))*cos(alpha);   z2 = z1b+(D(2)+Q(2))*sin(alpha);
    x3 = x2;        y3 = y2-A(3)*cos(alpha+Q(3));   z3 = z2+A(3)*sin(alpha+Q(3));
    
%     S = [S{:}]';
%     S = reshape(S,3,[]);

    figure(fig)
    hold on
    
    %     plot3(S(1,:),S(2,:),S(3,:),'.-k','LineWidth',2.5)   % To plot in one go
    
    plot3([x0 x1a],[y0 y1a],[z0 z1a],'.-r','LineWidth',2.5)
    plot3([x1a x1b],[y1a y1b],[z1a z1b],'.-r','LineWidth',2.5)
    plot3([x1b x2],[y1b y2],[z1b z2],'.-g','LineWidth',2.5)
    plot3([x2 x3],[y2 y3],[z2 z3],'.-b','LineWidth',2.5)
    
    hold off

end
