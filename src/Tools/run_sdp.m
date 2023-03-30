function [u_hat] = run_sdp(ctrl, x, u_prev, ref)
    % RUN_SDP utilizes semit-definite programming (SDP) relaxations to
    % solve for an approximatelly optimal input sequence.
    % Inputs:
    %   ctrl: Struct with all controller information. The controller is
    %     required to by of type 'n-step-SDP'.
    %   x: Current state of system.
    %   u_prev: Previously applied input u(k-1).
    %   ref: Torque- and absolute stator flux reference.
    % Outputs:
    %   u_hat: Approximated optimal input sequence.


    %% Assigning function parameters
    T_ref = ref(1);
    Psi_ref = ref(2);
    N = ctrl.N;
    sdp = ctrl.sdp;
    X = sdp.X;

    %% Cost function
    J = sdp.J0;
    % Add switching penalty to cost term
    J = J + ctrl.lam_u*norm(trace(sdp.Y0a(u_prev)*X),1);
    J = J + ctrl.lam_u*norm(trace(sdp.Y0b(u_prev)*X),1);
    J = J + ctrl.lam_u*norm(trace(sdp.Y0c(u_prev)*X),1);

    % Add tracking penalties to cost term
    D = [];
    for i = 0:N-1
        D = [
            D; 
            sqrt(ctrl.lam_T)*(T_ref - ctrl.T_factor*trace(sdp.Q(x,i)*X));
            sqrt(1-ctrl.lam_T)*(Psi_ref^2 - trace(sdp.W(x,i)*X))
        ];
    end
    J = J + norm(D,2)^2;
    
    %% Perform optimization
    assign(X,[1; ctrl.U_ed]*[1 ctrl.U_ed']);
    t2 = tic;
    optimize(sdp.F, J, sdp.sdpopt);
    t2 = toc(t2);

    %% Value estimation of x
    % Eigenvalue decomposition if needed to estimate x
    if ctrl.estimate == "all" || ctrl.estimate == "eigen vector" ...
            || ctrl.estimate == "eigen vector uniform"
        [V, Lambda] = eig(value(X));
        v1 = V(:,end)*sqrt(Lambda(end));
        v1 = sign(v1(1))*v1(2:end);
    end

    % Calculate approximated value of x
    switch ctrl.estimate
        case "first column"
            % Round first column
            u_hat = sign(value(X(1)))*round(value(X(2:end,1)));
        case "diagonal"
            % Round sqrt of diagonal
            u_hat = sign(value(X(:,1))).*...
                round(sqrt(diag(value(X(2:end,2:end)))));
        case "eigen vector"
            % Round first eigenvector normally
            u_hat = round(v1);
        case "eigen vector uniform"
            % Round first eigenvector with equal distribution over all
            % numbers (empirically showed to be slightly better than rest)
            u_hat = zeros(3*N,1);
            u_hat(v1 > 1/3) = 1;
            u_hat(v1 < -1/3) = -1;
        case "all"
            % Round first column
            u_hat1 = sign(value(X(1)))*round(value(X(2:end,1)));
            % Round sqrt of diagonal
            u_hat2 = sign(value(X(1)))*sign(value(X(2:end,1))).*...
                round(sqrt(diag(value(X(2:end,2:end)))));
            % Round first eigen vector normally
            u_hat3 = round(v1);
            % Round first eigen vector with equal distribution
            u_hat4 = zeros(3*N,1);
            u_hat4(v1 > 1/3) = 1;
            u_hat4(v1 < -1/3) = -1;
            % Return all estimates
            u_hat = [u_hat1 u_hat2 u_hat3 u_hat4];
    end

end