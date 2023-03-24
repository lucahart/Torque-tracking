function [U_opt, J_opt, iter, nodes] = branch_and_bound_nstep_SDP(J, J_opt, i, N, U, U_opt, x, u_prev, ref, iter, nodes, ctrl)



    % Terminate if node-limit is reached
    if nodes >= ctrl.node_limit
        return 
    else
        nodes = nodes + 1;
    end

    % Split up input-arrays into relevant variables
    psi_s = x(1:2);
    psi_r = x(3:4);
    T_ref = ref(1);
    Psi_ref = ref(2);
    
    % Create variables for faster execution during optimization
    A_1 = ctrl.A_1;
    B_1 = ctrl.B_1;
    B_2 = ctrl.B_2;
    T_factor = ctrl.T_factor;
    lam_T = ctrl.lam_T;
    lam_u = ctrl.lam_u;

    % Pre-calculation of variables that do not depend on the input u
    psi_r_kp1 = ctrl.A_2*psi_r + ctrl.B_3*psi_s;

    % Run optimization
    for u = ctrl.U_set
        % Apply dynamics depending on u and calculate associated cost J
        psi_s_kp1 = A_1*psi_s + B_1*psi_r + B_2*u;
        T_kp1 = T_factor*(psi_r_kp1(1)*psi_s_kp1(2) - psi_r_kp1(2)*psi_s_kp1(1));
        Psi_s_kp1 = norm(psi_s_kp1,2);
        J_prime = J + lam_T*norm(T_kp1-T_ref,2)^2 + (1-lam_T)*norm(Psi_s_kp1^2-Psi_ref^2,2)^2 + lam_u*norm(u_prev-u,1);
        % Advance in branche or update optimal cost if condition is satisfied
        if J_opt - J_prime > ctrl.eps
            if i < N
                U(3*(i-1)+1:3*i) = u;
                [U_opt, J_opt, iter, nodes] = branch_and_bound_nstep_SDP(J_prime, J_opt, i+1, N, U, U_opt, [psi_s_kp1;psi_r_kp1], u, ref, iter, nodes, ctrl);
            else
                U(3*(i-1)+1:3*i) = u;
                U_opt = U;
                J_opt = J_prime;
                iter = iter + 1;
            end
        end
    end

end