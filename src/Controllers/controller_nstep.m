function [u_opt, ctrl, iter] = controller_nstep(x, u_prev, ref, ctrl)
    % CONTROLLER_NSTEP is deprecated. It uses a full-horizon reference.
    % Please use CONTROLLER_NSTEP_SDP to only take the reference of k+1
    % into consideration. CONTROLLER_NSTEP_SDP is able to fully replace
    % this function.


    % Extend reference if array not long enough
    ref_length = min(ctrl.N,size(ref,2));
    if ref_length < ctrl.N
        ref = [ref ref(:,end).*ones(2,ctrl.N-ref_length)];
    end


    % Split up input-arrays into relevant variables
    psi_s = x(1:2);
    psi_r = x(3:4);
    T_ref = ref(1, 1:ctrl.N);
    Psi_ref = ref(2, 1:ctrl.N);

    % Create variables for faster execution
    A_1 = ctrl.A_1;
    A_2 = ctrl.A_2;
    B_1 = ctrl.B_1;
    B_2 = ctrl.B_2;
    B_3 = ctrl.B_3;
    T_factor = ctrl.T_factor;
    lam_T = ctrl.lam_T;
    lam_u = ctrl.lam_u;

    % Set initial values from educated guess
    U_opt = ctrl.U_ed;
    J_opt = 0;
    u_prev_temp = u_prev;
    psi_r_kp1 = psi_r;
    psi_s_kp1 = psi_s;
    for i = 1:ctrl.N
        u = U_opt(3*(i-1)+1:3*i);
        psi_r_kp1 = A_2*psi_r_kp1 + B_3*psi_s_kp1;
        psi_s_kp1 = A_1*psi_s_kp1 + B_1*psi_r_kp1 + B_2*u;
        T_kp1 = T_factor*(psi_r_kp1(1)*psi_s_kp1(2) - psi_r_kp1(2)*psi_s_kp1(1));
        Psi_s_kp1 = norm(psi_s_kp1,2);
        J_opt = J_opt + lam_T*(T_kp1-T_ref(i))^2 + (1-lam_T)*(Psi_s_kp1-Psi_ref(i))^2 + lam_u*norm(u_prev_temp-u,1);
        u_prev_temp = u;
    end

    % Set all other initial values
    iter = 0;
    U = U_opt;
    J = 0;

    % Run optimization
    [U_opt, ~, iter] = branch_and_bound_nstep(J, J_opt, 1, ctrl.N, U, U_opt, x, u_prev, ref(:,1:ctrl.N), iter, ctrl);

    % Update remaining variables
    u_opt = U_opt(1:3);
    if ctrl.N > 1
        ctrl.U_ed = [U_opt(4:end)', U_opt(3*ctrl.N-2:end)']';
    else
        ctrl.U_ed = U_opt;
    end

end