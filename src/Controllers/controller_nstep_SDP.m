function [u_opt, ctrl, iter, nodes, times, costs] = controller_nstep_SDP(x, u_prev, ref, ctrl, ex_sdp)


    
    iter = zeros(2,1);
    nodes = zeros(2,1);
    times = zeros(2,1);
    costs = zeros(6,1);
    if nargin < 5
        ex_sdp = 0;
    end
    
    %% SDP
    if ex_sdp
        % Run sdp to obtain guesses for u
        [u_hat] = run_sdp(ctrl,x,u_prev,ref(:,1));
        % Compute the cost of the guesses for u
        J_hat = cost(ctrl,x,u_prev,ref(:,1),u_hat);
        % Take cost J_sdp and input u_sdp of the best guess
        [J_sdp, idx] = min(J_hat);
        U_sdp = u_hat(:,idx);
        
        % Set all other initial values
        iter_sdp = 0;
        nodes_sdp = 0;
        U = U_sdp;
        J = 0;
        
        % Run optimization
        t = tic;
        [U_opt, J_opt, iter_sdp, nodes_sdp] = branch_and_bound_nstep_SDP(J, J_sdp, 1, ctrl.N, U, U_sdp, x, u_prev, ref(:,1), iter_sdp, nodes_sdp, ctrl);
        times(2) = toc(t);
        iter(2) = iter_sdp;
        nodes(2) = nodes_sdp;
        costs(3:6) = J_hat;
    end

    %% Educated Guess
    % Set initial values from educated guess
    U_ed = ctrl.U_ed;
    J_ed = cost(ctrl,x,u_prev,ref(:,1),U_ed);

    % Set all other initial values
    iter_ed = 0;
    nodes_ed = 0;
    U = U_ed;
    J = 0;

    % Run optimization
    t = tic;
    [U_opt, J_opt, iter_ed, nodes_ed] = branch_and_bound_nstep_SDP(J, J_ed, 1, ctrl.N, U, U_ed, x, u_prev, ref(:,1), iter_ed, nodes_ed, ctrl);
    times(1) = toc(t);
    iter(1) = iter_ed;
    nodes(1) = nodes_ed;
    costs(2) = J_ed;
    
    %% Post processing
    costs(1) = J_opt;

    % Update remaining variables
    u_opt = U_opt(1:3);
    if ctrl.N > 1
        ctrl.U_ed = [U_opt(4:end)', U_opt(3*ctrl.N-2:end)']';
    else
        ctrl.U_ed = U_opt;
    end

end