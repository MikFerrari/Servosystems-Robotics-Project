function [Q,q1,q2,q3] = DHinv(P,D,A,alpha)

    x=P(1); y=P(2); z=P(3);
    Q=[];

    % q1
    if y>=0 
        phi = atan(x/y);
    elseif y<0 
        phi = atan(x/y)+pi;
    end
    
    R=(x^2+y^2)^0.5; 
    q1 = pi-asin(A(1)/R)-phi;
    
    % q3
    if abs(q1-0.5*pi)<1e-10
        argument = (-x*sin(alpha)+(z-D(1))*cos(alpha))/A(3);
    elseif abs(q1-(-0.5*pi))<1e-10
        argument = (x*sin(alpha)+(z-D(1))*cos(alpha))/A(3);
    else
        argument = ((y-A(1)*sin(q1))./(cos(q1))*sin(alpha)+(z-D(1))*cos(alpha))/A(3);
    end
           
    q3(1) = asin(argument);
    if argument >= 0
                q3(2) = pi-asin(argument);
            else
                q3(2) = -pi-asin(argument);
    end
    
    % q2
    for i=1:length(q3)
        q2(i)=( z-D(1)-A(3)*sin(alpha+q3(i)) )/sin(alpha) - D(2);
        Q=[Q; q1 q2(i) q3(i)];
    end   