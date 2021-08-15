function [] = plot_robot_bold_2D(Q,L,angle,fig,dims)
% dims = dimension to plot the robot along.
%   1 = X;   2 = Y;   3 = Z
% E.G.: [1 2] = plane XY;

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
    
    if isequal(dims,[1 2])
        plot([x0 x1a],[y0 y1a],'.-r','LineWidth',2.5)
        plot([x1a x1b],[y1a y1b],'.-r','LineWidth',2.5)
        plot([x1b x1c],[y1b y1c],'.-r','LineWidth',2.5)
        plot([x1c x2],[y1c y2],'.-g','LineWidth',2.5)
        plot([x2 x3],[y2 y3],'.-b','LineWidth',2.5)

        % Plot Cw
        plot(x3,y3,'ok','MarkerSize',8)
        offset = 0.3;
        text(x3+offset,y3+offset,'C_w','Color','black','FontSize',14);
        hold off
    elseif isequal(dims,[1 3])
        plot([x0 x1a],[z0 z1a],'.-r','LineWidth',2.5)
        plot([x1a x1b],[z1a z1b],'.-r','LineWidth',2.5)
        plot([x1b x1c],[z1b z1c],'.-r','LineWidth',2.5)
        plot([x1c x2],[z1c z2],'.-g','LineWidth',2.5)
        plot([x2 x3],[z2 z3],'.-b','LineWidth',2.5)

        % Plot Cw
        plot(x3,z3,'ok','MarkerSize',8)
        offset = 0.3;
        text(x3+offset,z3+offset,'C_w','Color','black','FontSize',14);
        hold off
    elseif isequal(dims,[2 3])
        plot([y0 y1a],[z0 z1a],'.-r','LineWidth',2.5)
        plot([y1a y1b],[z1a z1b],'.-r','LineWidth',2.5)
        plot([y1b y1c],[z1b z1c],'.-r','LineWidth',2.5)
        plot([y1c y2],[z1c z2],'.-g','LineWidth',2.5)
        plot([y2 y3],[z2 z3],'.-b','LineWidth',2.5)

        % Plot Cw
        plot(y3,z3,'ok','MarkerSize',8)
        offset = 0.3;
        text(y3+offset,z3+offset,'C_w','Color','black','FontSize',14);
        hold off
    else
        error('Specify correct dimensions!')
    end

end

