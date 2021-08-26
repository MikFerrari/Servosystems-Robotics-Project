function prod = pseudoScalar(x,y)
    prod = x(1,4)*y(1,4) + x(2,4)*y(2,4) + x(3,4)*y(3,4) + ...
           x(1,3)*y(1,3) + x(2,1)*y(2,1) + x(3,2)*y(3,2);
    % prod = fx*vx + fy*vy + fz*vz + cx*wx + cy*wy + cz*wz
end

