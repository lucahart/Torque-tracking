function [u_opt, ctrl, iter, nodes, times, cost] = controller_1step(x, u_prev, ref, ctrl)
    % CONTROLLER_1STEP is a 1-step torque- and ablsolute stator flux
    % tracking ontroller with a prediction horizon of N = 1.
    % Inputs:
    %   x: Current state of system.
    %   u_prev: Previous contorl input u(k-1).
    %   ref: Torque- and absolute stator flux references.
    %   ctrl: Struct containing all information about the controller.
    % Outputs:
    %   u_opt: Control input u(k) that is optimal acctording to the 1-step
    %     MPC optimization problem.
    %   ctrl: Struct containing all information about the controller.
    %   iter: Does not have a meaning for N = 1. Default is -1.
    %   nodes: Doesn not have a meaning for N = 1. Default is -1.
    %   times: Time that the optimization takes.
    %   cost: Expected cost when applying the input u_opt according to the
    %     specified cost function.

    % Parameters to save measured data (Meaningless for N = 1)
    iter = -1;
    nodes = -1;

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
    t1 = tic;
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
    t1 = toc(t1);
    
    % Parameters to save measurement data (Meaningful for N = 1)
    cost = J_opt;
    times = t1;
end