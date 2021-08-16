function Q = inv_kin_OLD(S,L,angle,sol)

    l1a = L(1); l1b = L(2); l1c = L(3); l2 = L(4); l3 = L(5);
    
    x = S{1}; y = S{2}; z = S{3};
    
    % Compute q1 (needs only x and y)
    q1 = acos(l1b./sqrt(x.^2+y.^2))+atan2(y,x);
    
    % Compute q3 (needs x or y, z and q1)
    % There are 2 possible solutions
    
%     argument = ((y-l1b*sin(q1))./(cos(q1))*sin(angle)+(z-l1a)*cos(angle))/l3;

    T = sqrt(x.^2+y.^2);
    argument = ((z-l1a)*cos(angle)-T*sin(angle))/l3;

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
    
%     A = (z-l1a-tan(angle)*(x.*sin(q1)-y.*cos(q1)))/l3;
%     B = sqrt(1+tan(angle)^2);
%     if sol == 1
%         q3 = acos(A./B)-atan2(1,tan(angle))-angle;
%     elseif sol == 2
%         q3 = -acos(A./B)-atan2(1,tan(angle))-angle;
%     else
%         error("Only values 1 and 2 are possible for the parameter 'sol'")
%     end

    % Compute q2 (needs z and q3)
    % There is a unique solution, which hovewer depends on the choice of q3
    q2 = (z-l1a-l3*sin(angle+q3))/sin(angle)-l1c-l2;

    Q = {q1; real(q2); real(q3)};
    
end

