function Q = inv_kin_numerical(S,L,angle,Q0)

    fun = @(x) gripper_coord(S,L,angle,x);
    Q = fsolve(fun,[Q0{:}]');
    Q = num2cell(Q);
    
end

function F = gripper_coord(S,L,angle,t)

    l1a = L(1); l1b = L(2); l1c = L(3); l2 = L(4); l3 = L(5);
    
    x = S{1}; y = S{2}; z = S{3};
    
    F(1) = x-l1b*cos(t(1))-((l1c+l2+t(2))*cos(angle)+l3*cos(angle+t(3)))*sin(t(1));
    F(2) = y-l1b*sin(t(1))-((l1c+l2+t(2))*cos(angle)+l3*cos(angle+t(3)))*cos(t(1));
    F(3) = z-l1a-(l1c+l2+t(2))*sin(angle)-l3*sin(angle+t(3));

end