function p = DH_plot_robot(Q,D,A,alpha,fig,arg)

%     for i=1:size(Q,1)
%         for j=1:size(Q,2)
%             if Q(i,j)<0
%                 Q(i,j)=Q(i,j)+pi;
%             end
%         end
%     end
   
    x0 = 0;
    y0 = 0;
    z0 = 0;

    x1a = x0;
    y1a = y0;
    z1a = z0+D(1);

    x1b = x1a+A(1)*cos(Q(1));
    y1b = x1a+A(1)*sin(Q(1));
    z1b = z1a;

    x2 = x1b+(D(2)+Q(2))*cos(alpha).*sin(Q(1));
    y2 = y1b-(D(2)+Q(2))*cos(alpha).*cos(Q(1));
    z2 = z1b+(D(2)+Q(2))*sin(alpha);

    x3 = x2+A(3)*cos(alpha+Q(3)).*sin(Q(1));
    y3 = y2-A(3)*cos(alpha+Q(3)).*cos(Q(1));
    z3 = z2+A(3)*sin(alpha+Q(3));

    figure(fig)
    hold on
    
    if isa(arg,'char')==1
        col=arg;
        p = plot3([x0 x1a x1b x2 x3],[y0 y1a y1b y2 y3],[z0 z1a z1b z2 z3],'LineWidth',2,'color',col);   % Manipulator
%       p(2) = plot3(Pi(1),Pi(2),Pi(3),'*r');   % Joint position
        
    elseif isa(arg,'double')==1
        Pi=arg;
        plot3([x0 x1a],[y0 y1a],[z0 z1a],'.-r','LineWidth',2.5)
        plot3([x1a x1b],[y1a y1b],[z1a z1b],'.-r','LineWidth',2.5)
        plot3([x1b x2],[y1b y2],[z1b z2],'.-g','LineWidth',2.5)
        plot3([x2 x3],[y2 y3],[z2 z3],'.-b','LineWidth',2.5)
        plot3(Pi(1),Pi(2),Pi(3),'*r')
    end
    hold off

end
