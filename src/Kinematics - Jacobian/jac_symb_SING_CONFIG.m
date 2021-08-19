function [J_gripper_symb,J_complete_symb] = jac_symb_SING_CONFIG
% The workaround of the variable substitution is necessary, since MATLAB
% cannot perform jacobian with respect to symbolic functions (E.G.: x(t))

    [S_gripper_symb,S_complete_symb] = dir_kin_symb_SING_CONFIG;

    syms q1_sym q2_sym q3_sym
    J_gripper_symb = jacobian(S_gripper_symb,[q1_sym q2_sym q3_sym]);
    J_complete_symb = jacobian(S_complete_symb,[q1_sym q2_sym q3_sym]);

end