function p = plot_robot(Q,L,angle,fig)

    [~,S] = dir_kin(Q,L,angle);
    
    x0 = S{1};    y0 = S{2};    z0 = S{3};
    x1a = S{4};   y1a = S{5};   z1a = S{6};
    x1b = S{7};   y1b = S{8};   z1b = S{9};
    x1c = S{10};  y1c = S{11};  z1c = S{12};
    x2 = S{13};   y2 = S{14};   z2 = S{15};
    x3 = S{16};   y3 = S{17};   z3 = S{18};
    
    S = [S{:}]';
    S = reshape(S,3,[]);

    figure(fig)
    hold on
    
    %     plot3(S(1,:),S(2,:),S(3,:),'.-k','LineWidth',2.5)   % To plot in one go
    
    p(1) = plot3([x0 x1a],[y0 y1a],[z0 z1a],'.-r','LineWidth',2.5);
    p(2) = plot3([x1a x1b],[y1a y1b],[z1a z1b],'.-r','LineWidth',2.5);
    p(3) = plot3([x1b x1c],[y1b y1c],[z1b z1c],'.-r','LineWidth',2.5);
    p(4) = plot3([x1c x2],[y1c y2],[z1c z2],'.-g','LineWidth',2.5);
    p(5) = plot3([x2 x3],[y2 y3],[z2 z3],'.-b','LineWidth',2.5);
    
    hold off

end

