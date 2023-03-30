function [u_opt, ctrl, iter, nodes, times, costs] = controller_nstep(x, u_prev, ref, ctrl)
    % CONTROLLER_NSTEP is deprecated. It uses a full-horizon reference.
    % Please use CONTROLLER_NSTEP_SDP to only take the reference of k+1
    % into consideration. CONTROLLER_NSTEP_SDP is able to fully replace
    % this function.


    % Extend reference if array not long enough
    ref_length = min(ctrl.N,size(ref,2));
    if ref_length < ctrl.N
        ref = [ref ref(:,end).*ones(2,ctrl.N-ref_length)];
    end

    % Set initial values from educated guess
    U_ed = ctrl.U_ed;
    J_ed = cost(ctrl,x,u_prev,ref,U_ed);

    % Set all other initial values
    iter = 0;
    U = U_ed;
    J = 0;

    % Run optimization
    t = tic;
    [U_opt, J_opt, iter] = branch_and_bound_nstep(J, J_ed, 1, ctrl.N, U, U_ed, x, u_prev, ref(:,1:ctrl.N), iter, ctrl);
    times = toc(t);
    
    costs = [J_opt;J_ed];
    nodes = -1; % no measurements taken, use SDP ctrler for this information

    % Update remaining variables
    u_opt = U_opt(1:3);
    if ctrl.N > 1
        ctrl.U_ed = [U_opt(4:end)', U_opt(3*ctrl.N-2:end)']';
    else
        ctrl.U_ed = U_opt;
    end

end