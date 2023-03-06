function [ctrl, run_ctrl] = controllerSetup(sys, ctrl_type, ctrl_pre)
    


    % *********************************************************************
    %% Make changes to this section (controller variables)
    % *********************************************************************

    % 1-step controller
    ctrl_1step.lam_u = 1.2e-4; %.198e-3;
    ctrl_1step.lam_T = .052;
    ctrl_1step.T_s = 25e-6;
    ctrl_1step.N = 1;

    % Multi-step controller
    ctrl_nstep.lam_u = 6.15e-4; %3e-3; % 1.15e-4 if lam_T = .1 and 2.7e-4 if lam_T = .5 to hold 230-240Hz
    ctrl_nstep.lam_T = .052;
    ctrl_nstep.T_s = 25e-6;
    ctrl_nstep.N = 3;

    % Multi-step controller with speed-up




    % *********************************************************************
    %% Do not make changes to this section
    % *********************************************************************

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Errors
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if nargin <= 1
        error('System and controller type are required.');
    end

    if ctrl_type ~= "1-step" && ...
            ctrl_type ~= "n-step" && ...
            ctrl_type ~= "n-step+su"
        error('Choose a valid controller type.');
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % 1-step controller
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if ctrl_type == "1-step"
        % Set all previously specified parameters
        ctrl = ctrl_1step;

        % Calculate discrete-time matrices
        ctrl.A_1 = eye(2) + sys.F_1*ctrl.T_s*sys.w_base;
        ctrl.A_2 = eye(2) + sys.F_2*ctrl.T_s*sys.w_base;
        ctrl.B_1 = sys.G_1*ctrl.T_s*sys.w_base;
        ctrl.B_2 = sys.G_2*ctrl.T_s*sys.w_base;
        ctrl.B_3 = sys.G_3*ctrl.T_s*sys.w_base;
        ctrl.B_4 = sys.G_4*ctrl.T_s*sys.w_base;

        % Generate the set U = {-1,0,1}^3
        u = [-1, 0, 1];
        [U1, U2, U3] = ndgrid(u, u, u);
        ctrl.U_set = [U1(:), U2(:), U3(:)]';

        % Set the run function that executes the control algorithm in
        % simulation
        run_ctrl = @controller_1step;
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Multi-step controller
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if ctrl_type == "n-step"
        % Set all previously specified parameters
        ctrl = ctrl_nstep;

        % Calculate discrete-time matrices
        ctrl.A_1 = eye(2) + sys.F_1*ctrl.T_s*sys.w_base;
        ctrl.A_2 = eye(2) + sys.F_2*ctrl.T_s*sys.w_base;
        ctrl.B_1 = sys.G_1*ctrl.T_s*sys.w_base;
        ctrl.B_2 = sys.G_2*ctrl.T_s*sys.w_base;
        ctrl.B_3 = sys.G_3*ctrl.T_s*sys.w_base;
        ctrl.B_4 = sys.G_4*ctrl.T_s*sys.w_base;

        % Generate the set U = {-1,0,1}^3
        u = [-1, 0, 1];
        [U1, U2, U3] = ndgrid(u, u, u);
        ctrl.U_set = [U1(:), U2(:), U3(:)]';

        % Set the run function that executes the control algorithm in
        % simulation
        run_ctrl = @controller_nstep;
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Multi-step controller with speed-up
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if ctrl_type == "n-step+su"
        ctrl = ctrl_nstepsu;
    end  

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Pre-specified controller parameters
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Overwrites the default controller parameters of the section above if
    % there are pre-specified controller parameters
    if nargin >= 3
        fn = fieldnames(ctrl_pre);
        for i = 1:numel(fn)
            ctrl.(fn{i}) = ctrl_pre.(fn{i});
        end
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Useful for all controllers
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ctrl.T_factor = sys.X_m/(sys.D*sys.pf);
    ctrl.U_ed = zeros(3*ctrl.N,1);
    
end