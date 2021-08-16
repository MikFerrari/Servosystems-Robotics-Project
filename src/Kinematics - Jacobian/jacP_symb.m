function [Jp_gripper_symb,Jp_complete_symb] = jacP_symb
     
    [J_gripper_symb,J_complete_symb] = jac_symb;

    syms t q1_sym(t) q2_sym(t) q3_sym(t) q1p_sym(t) q2p_sym(t) q3p_sym(t)

    Jp_gripper_symb = diff(J_gripper_symb,t);
    Jp_gripper_symb = subs(Jp_gripper_symb, ...
                           [diff(q1_sym(t), t), diff(q2_sym(t), t), diff(q3_sym(t), t)], ...
                           [q1p_sym(t) q2p_sym(t) q3p_sym(t)]);
                       
    syms t q1_sym(t) q2_sym(t) q3_sym(t) q1p_sym(t) q2p_sym(t) q3p_sym(t)

    Jp_complete_symb = diff(J_complete_symb,t);
    Jp_complete_symb = subs(Jp_complete_symb, ...
                           [diff(q1_sym(t), t), diff(q2_sym(t), t), diff(q3_sym(t), t)], ...
                           [q1p_sym(t) q2p_sym(t) q3p_sym(t)]);

end

