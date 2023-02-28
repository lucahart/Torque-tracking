function [u_opt, ctrl, iter] = controller_1step(x, u_prev, ref, ctrl)



    % Split up input-arrays into relevant variables
    psi_s = x(1:2);
    psi_r = x(3:4);
    T_ref = ref(1,1);
    Psi_ref = ref(2,1);

    % Create variables for faster execution during optimization
    A_1 = ctrl.A_1;
    B_1 = ctrl.B_1;
    B_2 = ctrl.B_2;
    T_factor = ctrl.T_factor;
    lam_T = ctrl.lam_T;
    lam_u = ctrl.lam_u;

    % Pre-calculation of variables that do not depend on the input u
    psi_r_kp1 = ctrl.A_2*psi_r + ctrl.B_3*psi_s;

    % Initialize optimal values
    J_opt = inf;
    u_opt = u_prev;

    % Run optimization
    for u = ctrl.U_set
        % Apply dynamics depending on u and calculate associated cost J
        psi_s_kp1 = A_1*psi_s + B_1*psi_r + B_2*u;
        T_kp1 = T_factor*(psi_r_kp1(1)*psi_s_kp1(2) - psi_r_kp1(2)*psi_s_kp1(1));
        Psi_s_kp1 = norm(psi_s_kp1,2);
        J = lam_T*(T_kp1-T_ref)^2 + (1-lam_T)*(Psi_s_kp1-Psi_ref)^2 + lam_u*norm(u_prev-u,1);
        % Update optimal values
        if J < J_opt
            J_opt = J;
            u_opt = u;
        end
    end
    
    % Assign -1 iterations as this metric is not useful for the 1-step
    % decoder
    iter = -1;
end