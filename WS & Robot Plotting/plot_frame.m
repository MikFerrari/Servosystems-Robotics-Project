function [] = plot_frame(R,T,scaling_factor)

    x = T(1); y = T(2); z = T(3);
    u = R*[1;0;0]/scaling_factor;
    v = R*[0;1;0]/scaling_factor;
    w = R*[0;0;1]/scaling_factor;
    
    hold on
    quiver3(x,y,z,u(1),u(2),u(3),'r-')
    quiver3(x,y,z,v(1),v(2),v(3),'g-')
    quiver3(x,y,z,w(1),w(2),w(3),'b-')
    hold off

end

