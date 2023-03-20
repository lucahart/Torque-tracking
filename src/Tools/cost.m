function J = cost(ctrl, x, u_prev, ref, U)

%TODO: Implement that you can give multiple U, e.g. U = [u_hat1; u_hat2],
%would return a vector of costs J = [J_hat1, J_hat2].

%     % Split up input-arrays into relevant variables
%     psi_s = x(1:2);
%     psi_r = x(3:4);
%     T_ref = ref(1,1);%ref(1, 1:ctrl.N);
%     Psi_ref = ref(2,1);% 1:ctrl.N);
% 
%     % Create variables for faster execution
%     A_1 = ctrl.A_1;
%     A_2 = ctrl.A_2;
%     B_1 = ctrl.B_1;
%     B_2 = ctrl.B_2;
%     B_3 = ctrl.B_3;
%     T_factor = ctrl.T_factor;
%     lam_T = ctrl.lam_T;
%     lam_u = ctrl.lam_u;

    J = zeros(1,size(U,2));
    psi_r = x(3:4).*ones(2,size(U,2));
    psi_s = x(1:2).*ones(2,size(U,2));
    for i = 1:ctrl.N
        u = U(3*(i-1)+1:3*i,:);
        psi_r = ctrl.A_2*psi_r + ctrl.B_3*psi_s;
        psi_s = ctrl.A_1*psi_s + ctrl.B_1*psi_r + ctrl.B_2*u;
        T_kp1 = ctrl.T_factor*(psi_r(1,:).*psi_s(2,:) - psi_r(2,:).*psi_s(1,:));
        Psi_s_kp1 = sqrt(sum(psi_s.^2,1)); % column-wise l2-norm
        J = J + ctrl.lam_T*(ref(1) - T_kp1).^2 + ...
            (1-ctrl.lam_T)*(ref(2)^2 - Psi_s_kp1.^2).^2 + ...
            ctrl.lam_u*sum(abs(u_prev-u),1);
        u_prev = u;
    end





end