function Q = inv_kin(S,L,angle,sol)

    l1a = L(1); l1b = L(2); l1c = L(3); l2 = L(4); l3 = L(5);
    
    x = S{1}; y = S{2}; z = S{3};
    
    % Compute q1 (needs only x and y)
    if x >= 0 && y >= 0
        q1 = 0.5*asin((l1b^2-x.^2-y.^2)./(x.*y));
    elseif x < 0 && y >= 0
        q1 = 0.5*pi-0.5*asin((l1b^2-x.^2-y.^2)./(x.*y));
    elseif x < 0 && y < 0
        q1 = -0.5*pi+0.5*asin((l1b^2-x.^2-y.^2)./(x.*y));
    elseif x >= 0 && y < 0
        q1 = 0.5*asin((l1b^2-x.^2-y.^2)./(x.*y));
    elseif x == 0 && y == 0
        q1 = 0; % q1 is not defined (it is set to 0 by default)
    end
    
    % Compute q3 (needs x or y, z and q1)
    % There are 2 possible solutions:
    % sol 1: q3 >= 0
    % sol 2: q3 < 0
    argument = ((y-l1b*sin(q1))./(cos(q1))*sin(angle)+(z-l1a)*cos(angle))/l3;
    if sol == 1
        q3 = asin(argument);
    elseif sol == 2
        if argument >= 0
            q3 = pi-asin(argument);
        else
            q3 = -pi-asin(argument);
        end
    else
        error("Only values 1 and 2 are possible for the parameter 'sol'")
    end
    
    % Compute q2 (needs z and q3)
    % There is a unique solution, which hovewer depends on the choice of q3
    q2 = (z-l1a-l3*sin(angle+q3))/(sin(angle))-l1c-l2;
    
    Q = {q1; q2; q3};
    
end

