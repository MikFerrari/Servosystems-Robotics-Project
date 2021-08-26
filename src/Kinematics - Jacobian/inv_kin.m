function Q = inv_kin(S,L,angle,sol)

    l1a = L(1); l1b = L(2); l1c = L(3); l2 = L(4); l3 = L(5);
    
    x = S{1}; y = S{2}; z = S{3};
    
    % Compute q1 (needs only x and y)
    if y >= 0 
        phi = atan(x/y);
    elseif y < 0 
        phi = atan(x/y)+pi;
    end
    
    R = (x^2+y^2)^0.5; 
    q1 = pi-asin(l1b/R)-phi;
    
    % Compute q3 (needs x or y, z and q1)
    % There are 2 possible solutions
    if abs(q1-0.5*pi) < 1e-10
        argument = (-x*sin(angle)+(z-l1a)*cos(angle))/l3;
    elseif abs(q1-(-0.5*pi)) < 1e-10
        argument = (x*sin(angle)+(z-l1a)*cos(angle))/l3;
    else
        argument = ((y-l1b*sin(q1))./(cos(q1))*sin(angle)+(z-l1a)*cos(angle))/l3;
    end
    
    if sol == 1
        q3 = asin(argument);
    elseif sol == 2
        if argument >= 0
            q3 = pi-asin(argument);
        else
            q3 = -pi-asin(argument);
        end
    end
    
    % Compute q2 (needs z and q3)
    % There is a unique solution, which hovewer depends on the choice of q3
    q2 = ( z-l1a-l3*sin(angle+q3) )/sin(angle) - l1c-l2;
    
    Q = {q1; real(q2); real(q3)};
        
end

