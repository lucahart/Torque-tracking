function J = cost(ctrl, x, u_prev, ref, U)
    % COST computes the cost of applying an input sequence U at the
    % specified state.
    % Input:
    %   ctrl: Struct that contains all information about the controller.
    %   x: Current state of system.
    %   u_prev: Previous control input u(k-1).
    %   ref: Torque- and absolute stator flux reference.
    %   U: Input sequence to compute the cost for.

    J = zeros(1,size(U,2));
    psi_r = x(3:4).*ones(2,size(U,2));
    psi_s = x(1:2).*ones(2,size(U,2));
    for i = 1:ctrl.N
        u = U(3*(i-1)+1:3*i,:);
        psi_r_kp1 = ctrl.A_2*psi_r + ctrl.B_3*psi_s;
        psi_s_kp1 = ctrl.A_1*psi_s + ctrl.B_1*psi_r + ctrl.B_2*u;
        psi_r = psi_r_kp1;
        psi_s = psi_s_kp1;
        T_kp1 = ctrl.T_factor*(psi_r(1,:).*psi_s(2,:) - psi_r(2,:).*psi_s(1,:));
        Psi_s_kp1 = sqrt(sum(psi_s.^2,1)); % column-wise l2-norm
        J = J + ctrl.lam_T*(ref(1) - T_kp1).^2 + ...
            (1-ctrl.lam_T)*(ref(2)^2 - Psi_s_kp1.^2).^2 + ...
            ctrl.lam_u*sum(abs(u_prev-u),1);
        u_prev = u;
    end
end