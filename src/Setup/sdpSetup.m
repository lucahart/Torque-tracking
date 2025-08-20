function sdp = sdpSetup(ctrl)
    % SDPSETUP sets up a struct containing all inforamtion about the
    % semi-definite programming (SDP) relaxation.
    %
    % Input:
    %   ctrl: Struct contraining all information abou the controller (See
    %     ctrlSetup).
    % Output:
    %   sdp: Struct containing all precalculations and parameters to speed
    %     up the solving process of the SDP.
    %
    % Parameters:
    %   epsilon: Maximum inprecision of solution.
    %     Default: epsilon = 1e-4
    %   sdp.sdpopt: Different settings of the optimizer.
    
    
    % *********************************************************************
    %% Make changes to this section (SDP settings)
    % *********************************************************************
    
    epsilon = 1e-4; % allowed imprecision of solution
    sdp.sdpopt = sdpsettings(...
        'solver', 'scs', ...
        'verbose', 0, ...
        'dualize', 0, ...
        'scs.normalize', 1, ...
        'scs.max_iters', 120, ...
        'scs.eps_abs', epsilon, ...
        'scs.eps_rel', epsilon ...
    );

    
    % *********************************************************************
    %% Do not make changes to this section
    % *********************************************************************

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Required parameters
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    % Discrete system dynamics
    A = [
        ctrl.A_1 ctrl.B_1;
        ctrl.B_3 ctrl.A_2;
    ];
    B = [
        ctrl.B_2;
        ctrl.B_4;
    ];

    % System dymension and control horizon
    n_x = 4;
    N = ctrl.N;

    
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Matrix calculations
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    % Stator flux dynamics: psi_s(k+l) = Gamma_s*x(k) + Upsilon_s*U(k)
    Gamma_s = zeros(2*N,n_x);
    for i = 0:N-1
        Gamma_s(2*i + 1:2*(i+1), :) = A(1:2,:)*A^i;
    end

    Upsilon_s = zeros(2*N,3*N);
    for i = 0:N-1
        for j = 0:i
            if i == j
                Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = ctrl.B_2;
            else
                Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A(1:2,:)*A^(i-j-1)*B;
            end
        end
    end

    % Rotor flux dynamics: psi_r(k+l) = Gamma_r*x(k) + Upsilon_r*U(k)
    Gamma_r = zeros(2*N,n_x);
    for i = 0:N-1
        Gamma_r(2*i + 1:2*(i+1), :) = A(3:4,:)*A^i;
    end

    Upsilon_r = zeros(2*N,3*N);
    for i = 0:N-1
        for j = 0:i-1
            Upsilon_r(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A(3:4,:)*A^(i-j-1)*B;
        end
    end

    % Cross-product for torque calculation: crossprod(a,b) = a'*Zeta*b
    Zeta = [0 1; -1 0];

    
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Polynomial representations
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    % Torque: T(k+l) = trace(Q(x,l-1)*X), X = U(k)*U(k)'
    sdp.Q = @(x,i) [
        x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, ...
        x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:);
        Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, ...
        Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:)
    ];

    % Absolute stator flux: Psi_s(k+l) = trace(W(x,l-1)*X), X = U(k)*U(k)'
    sdp.W = @(x,i) [
        x'*Gamma_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, ...
        x'*Gamma_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:);
        Upsilon_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, ...
        Upsilon_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:)
    ];

    % Switching penalty depending on u_prev: \Del u(k) = u(k) - u_prev
    sdp.Y0a = @(u_prev) [-u_prev(1) 1 zeros(1,3*N-1); zeros(3*N,3*N+1)];
    sdp.Y0b = @(u_prev) [-u_prev(2) 0 1 zeros(1,3*N-2); zeros(3*N,3*N+1)];
    sdp.Y0c = @(u_prev) [-u_prev(3) 0 0 1 zeros(1,3*N-3); zeros(3*N,3*N+1)];

    
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Setup SDP problem
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    % Clear all previous variables
    yalmip('clear');
    
    % Create sdpvar object
    sdp.X = sdpvar(3*N+1);

    % Create constraint set with all convex constraints of the SDP
    % relaxation:
    % PSD matrix and constant multiplier to 1
    sdp.F = [
        sdp.X >= 0,
        sdp.X(1) == 1,
    ];
    % Constraint on diagonal terms
    sdp.F = [
        sdp.F, 
        0 <= diag(sdp.X) <= 1
    ];
    % Constraint on off-diagonal terms (only holds for lb=-1 and ub=1)
    sdp.F = [
        sdp.F, 
        -1 <= sdp.X(logical(triu(true(3*N+1))-diag(true(3*N+1,1)))) <= 1
    ];

    % Create part of the cost function that is not dependent on the state x
    % or the previous input u_prev
    sdp.J0 = 0;
    for i = 2:3*N-2
        Y = [zeros(1,i-1), -1, zeros(1,2), 1, zeros(1, 3*N+1-(i+3)); zeros(3*N,3*N+1)];
        sdp.J0 = sdp.J0 + ctrl.lam_u*abs(trace(Y*sdp.X));
    end

end