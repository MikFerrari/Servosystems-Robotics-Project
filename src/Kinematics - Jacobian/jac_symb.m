function [J_gripper_symb,J_complete_symb] = jac_symb
% The workaround of the variable substitution is necessary, since MATLAB
% cannot perform jacobian with respect to symbolic functions (E.G.: x(t))

    [S_gripper_symb,S_complete_symb] = dir_kin_symb;

    syms t q1_sym(t) q2_sym(t) q3_sym(t) temp1 temp2 temp3
    
    J_grip_subs = subs(S_gripper_symb,{q1_sym q2_sym q3_sym},{temp1 temp2 temp3});
    J_gripper_symb = jacobian(J_grip_subs,[temp1 temp2 temp3]);
    J_gripper_symb = subs(J_gripper_symb,{temp1 temp2 temp3},{q1_sym q2_sym q3_sym});
    
    syms t q1_sym(t) q2_sym(t) q3_sym(t) temp1 temp2 temp3
    
    J_compl_subs = subs(S_complete_symb,{q1_sym q2_sym q3_sym},{temp1 temp2 temp3});
    J_complete_symb = jacobian(J_compl_subs,[temp1 temp2 temp3]);
    J_complete_symb = subs(J_complete_symb,{temp1 temp2 temp3},{q1_sym q2_sym q3_sym});

end