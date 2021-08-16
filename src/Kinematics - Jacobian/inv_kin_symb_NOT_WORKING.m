function SOL = inv_kin_symb_NOT_WORKING
    
    [S_gripper_symb,~] = dir_kin_symb;

    syms t q1_sym(t) q2_sym(t) q3_sym(t) temp1 temp2 temp3

    S_grip_subs = subs(S_gripper_symb,{q1_sym q2_sym q3_sym},{temp1 temp2 temp3});

    syms XE YE ZE

    S = S_grip_subs(1);
    XE_RHS = S(1);  YE_RHS = S(2);  ZE_RHS = S(3);

    XE_EQ = XE == XE_RHS;
    YE_EQ = YE == YE_RHS;
    ZE_EQ = ZE == ZE_RHS;


    SOL = solve([XE_EQ YE_EQ ZE_EQ], [temp1 temp2 temp3]);

    % XE_EQ = subs(XE_EQ,{temp1 temp2 temp3},{q1_sym q2_sym q3_sym});
    % SOL = solve([XE_EQ YE_EQ ZE_EQ], [q1_sym q2_sym q3_sym])

    simplify(SOL.q1_sym);
    simplify(SOL.q2_sym);
    simplify(SOL.q3_sym);
    
end

