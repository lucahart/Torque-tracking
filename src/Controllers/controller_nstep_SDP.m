function [u_opt, ctrl, iter, nodes, times, costs] = controller_nstep_SDP(x, u_prev, ref, ctrl)



    % Initialize measurement variables (except for costs)
    iter = zeros(1,1);
    nodes = zeros(1,1);
    times = zeros(1,1);
    
    %% Run controller
    if ctrl.type == "ed guess + sdp"
        costs = zeros(3,1); % J_bnb, J_ed, J_sdp
        
        % Set initial values from educated guess
        U_ed = ctrl.U_ed;
        J_ed = cost(ctrl,x,u_prev,ref(:,1),U_ed);

        % Set all other initial values
        iter_bnb = 0;
        nodes_bnb = 0;
        U = U_ed;
        J = 0;

        % Run optimization
        t = tic;
        [U_bnb, J_bnb, iter_bnb, nodes_bnb] = branch_and_bound_nstep_SDP(J, J_ed, 1, ctrl.N, U, U_ed, x, u_prev, ref(:,1), iter_bnb, nodes_bnb, ctrl);
        times(1) = toc(t);
        
        if nodes_bnb >= ctrl.node_limit
            % Run sdp to obtain guesses for U and get educated guess
            U_hat = run_sdp(ctrl,x,u_prev,ref(:,1));
            % Compute the cost of the guesses for U
            J_hat = cost(ctrl,x,u_prev,ref(:,1),U_hat);
            % Take cost J_sdp and input U_sdp of the best guess
            [J_sdp, idx] = min(J_hat);
            U_sdp = U_hat(:,idx);
            % Take best guess
            if J_sdp < J_bnb
                disp('Used SDP');
                U_opt = U_sdp;
                J_opt = J_sdp;
            else
                U_opt = U_bnb;
                J_opt = J_bnb;
            end
            costs(3) = J_sdp';
        else
            U_opt = U_bnb;
            J_opt = J_bnb;
            costs(3) = NaN(1,1);
        end
        iter(1) = iter_bnb;
        nodes(1) = nodes_bnb;
        costs(1) = J_bnb;
        costs(2) = J_ed;
        
    elseif ctrl.type == "ed & sdp guess"
        costs = zeros(6,1); % J_opt, J_ed, J_'first col', J_'diag', J_'eig vec', J_'eig vec uniform'
        
        % Run sdp to obtain guesses for U and get educated guess
        u_hat = [ctrl.U_ed run_sdp(ctrl,x,u_prev,ref(:,1))];
        % Compute the cost of the guesses for U
        J_hat = cost(ctrl,x,u_prev,ref(:,1),u_hat);
        % Take cost J_sdp and input U_sdp of the best guess
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
        costs(1) = J_opt;
        costs(2:6) = J_hat;
        
    elseif ctrl.type == "ed guess"
        costs = zeros(2,1); % J_opt, J_ed
        
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
        costs(1) = J_opt;
        costs(2) = J_ed;
    end
    
    %% Post processing
    % Update remaining variables
    u_opt = U_opt(1:3);
    if ctrl.N > 1
        ctrl.U_ed = [U_opt(4:end)', U_opt(3*ctrl.N-2:end)']';
    else
        ctrl.U_ed = U_opt;
    end

end