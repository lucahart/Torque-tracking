function [ctrl, run_ctrl] = ControllerSetup(sys, ctrl_type, ctrl_pre)
    % CONTROLLERSETUP sets up a struct containing all inforamtion about the
    % controller.
    %
    % Inputs:
    %   sys: System that is to be controlled.
    %   ctrl_type: Type of controller. Possible values are '1-step' for a
    %     controller with a horizon of N=1, 'n-step' for a controller with
    %     horizon N>1, and 'n-step-SDP' for a controller with horizon N>1 
    %     and the possibility to use SDPs.
    %   ctrl_pre (optional): Pre-set controller parameters. Everything that
    %     is set in this struct will overwrite the default parameters. All
    %     possible parameters and their default values are listed below.
    % Outputs:
    %   ctrl: Struct containing all information about the controller.
    %   run_ctrl: Function handle that can be executed to run the
    %     controller.
    %
    % The following parametes are specified for all contorller types:
    %   ctrl.lam_u: Switching penalty of the cost function. 
    %     lam_u \in [0,inf).
    %     Default: lam_u = 1.2e-4 for N=1, lam_u = 1.8e-3 for N=5
    %   ctrl.lam_T: Torque- to absolute stator flux penalty ratio.
    %     lam_T \in [0,1].
    %     Default: lam_T = .052
    %   ctrl.T_s: Sampling time of the controller [s].
    %     Default: T_s = 25e-6
    %   ctrl.N: Control horizon. For ctrl_type = '1-step' it always holds
    %     that N = 1. N is a positive integer otherwise.
    %     Default: N = 1 for ctrl_type='1-step' and N = 5 otherwise
    %
    % The following parameters are only specified for the controller type
    % 'n-step-SDP':
    %   ctrl.node_limit: A positive integer number that specifies the
    %     maximum number of nodes that the branch-and-bound algorithm is
    %     allowed to traverse when it solves the MPC's optimization
    %     problem.
    %     NOTE: For all values ctrl.node_limit < inf the solution
    %     is not guaranteed to be optimal.
    %     Default: node_limit = 100
    %   ctrl.type: Specifies the type of controller that is executed.
    %     'ed guess' uses a conventional branch-and-bound algorithm for 
    %     solving the underlying problem.
    %     'ed guess + sdp' executes the same algorithm as in 'ed guess'. 
    %     Additionally, an SDP relaxation of the problem is computed if the
    %     branch-and-bound algorithm reaches the ctrl.node_limit and 
    %     terminates. The best estimate of the 'ed guess' algorithm and SDP
    %     relaxation is applied.
    %     'ed & sdp guess' computes the SDP relaxation and compares it with
    %     the educated guess. The solution with the lowest cost is used as 
    %     initial guess for the branch-and-bound algorithm.
    %     Default: type = 'ed guess'
    %   ctrl.estimate: Only relevant when ctrl.type \in {'ed guess + sdp',
    %     'ed & sdp guess'}. Specifies how the optimal solution is computed
    %     after solving the SDP relaxation.
    %     'first column' rounds the first column of the solution matrix X.
    %     'diagonal' rounds the sqrt of diag(X).
    %     'eigen vector' applies an eigen value decomposition, takes the
    %       eigenvector v_1 with the greatest eigen value lam_1, and
    %       returns round(sqrt(lam_1)*v_1).
    %     'eigen vector uniform' applies 'eigen vector' but rounds at -1/3
    %     and 1/3 respectively to achieve a uniform distribution of values.
    %     'all' computes all values and returns a matrix with all solutions
    %     as vectors
    %     Default: estimate = 'first column'.
    %   ctrl.eps: Allowed numeric inprecision. It must hold that 
    %     J_opt - J_prime > eps for updating U_opt in the branch-and-bound
    %     algorithm
    %     Default: eps = 1e-12.
    %   ctrl.verbose: Prints whether the SDP was used when set to 1. No
    %     printing if volatile = 0.
    %     Default: volatile = 0.
    %   ctrl.deactivate: Deactivates the controller. When executing the
    %     controller function handle the controller will not start but
    %     return only default values. Default values are 0 for all
    %     measurements and NaN for the controller inputs. Can be used to
    %     spontaniously remove controllers out of the simulation without
    %     changing the whole simulation setting.
    %     Default: dectivate = 0.


    % *********************************************************************
    %% Default controller parameters (Make changes to this section)
    % *********************************************************************

    % 1-step controller
    ctrl_1step.lam_u = 5e-4;
    ctrl_1step.lam_T = .052;
    ctrl_1step.T_s = 25e-6;
    ctrl_1step.N = 1;

    % Multi-step controller
    ctrl_nstep.lam_u = 6.15e-4;
    ctrl_nstep.lam_T = .052;
    ctrl_nstep.T_s = 25e-6;
    ctrl_nstep.N = 5;

    % Multi-step controller with speed-up
    ctrl_nstep_SDP.lam_u = 3.8e-3; % use lam_u = 13e-3, 21e-3, 21e-3,8e-3 at 
                                  % T_s = 25e-6, 50e-6, 75e-6, 100e-6 for
                                  % f_sw = 226Hz (yes, it's veeery nonlin.)
    ctrl_nstep_SDP.lam_T = .052;
    ctrl_nstep_SDP.T_s = 25e-6;
    ctrl_nstep_SDP.N = 5;
    ctrl_nstep_SDP.node_limit = 250;
    ctrl_nstep_SDP.estimate = 'first column';
    ctrl_nstep_SDP.type = 'ed guess';
    ctrl_nstep_SDP.eps = 1e-12;
    ctrl_nstep_SDP.verbose = 0;
    ctrl_nstep_SDP.deactivate = 0;

    
    % *********************************************************************
    %% Do NOT make changes to this section
    % *********************************************************************

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Errors
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if nargin <= 1
        error('System and controller type are required.');
    end

    if ctrl_type ~= "1-step" && ...
            ctrl_type ~= "n-step" && ...
            ctrl_type ~= "n-step-SDP"
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

        % Set the run function that executes the control algorithm in
        % simulation
        run_ctrl = @controller_1step;
        
        % Specify length of cost term
        ctrl.n_costs = 1;
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Multi-step controller
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if ctrl_type == "n-step"
        
        warning("The 'n-step' controller is deprecated. " + ...
        "Please use the 'n-step-SDP' controller with " + ...
        "ctrl.type = 'ed guess' instead.");
        
        % Set all previously specified parameters
        ctrl = ctrl_nstep;

        % Calculate discrete-time matrices
        ctrl.A_1 = eye(2) + sys.F_1*ctrl.T_s*sys.w_base;
        ctrl.A_2 = eye(2) + sys.F_2*ctrl.T_s*sys.w_base;
        ctrl.B_1 = sys.G_1*ctrl.T_s*sys.w_base;
        ctrl.B_2 = sys.G_2*ctrl.T_s*sys.w_base;
        ctrl.B_3 = sys.G_3*ctrl.T_s*sys.w_base;
        ctrl.B_4 = sys.G_4*ctrl.T_s*sys.w_base;

        % Set the run function that executes the control algorithm in
        % simulation
        run_ctrl = @controller_nstep;
        
        % Specify length of cost term
        ctrl.n_costs = 2;
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Multi-step controller with SDP
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    
    if ctrl_type == "n-step-SDP"
        % Set all previously specified parameters
        ctrl = ctrl_nstep_SDP;
        
        % Overwrites the default controller parameters of the section above
        % if there are pre-set controller parameters in ctrl_pre
        if nargin >= 3
            fn = fieldnames(ctrl_pre);
            for i = 1:numel(fn)
                ctrl.(fn{i}) = ctrl_pre.(fn{i});
            end
        end
        
        % Calculate discrete-time matrices
        ctrl.A_1 = eye(2) + sys.F_1*ctrl.T_s*sys.w_base;
        ctrl.A_2 = eye(2) + sys.F_2*ctrl.T_s*sys.w_base;
        ctrl.B_1 = sys.G_1*ctrl.T_s*sys.w_base;
        ctrl.B_2 = sys.G_2*ctrl.T_s*sys.w_base;
        ctrl.B_3 = sys.G_3*ctrl.T_s*sys.w_base;
        ctrl.B_4 = sys.G_4*ctrl.T_s*sys.w_base;

        % Set the run function that executes the control algorithm in
        % simulation
        run_ctrl = @controller_nstep_SDP;
        
        % SDP setup
        ctrl.sdp = sdpSetup(ctrl);
        
        % Specify BASE LENGTH of cost term
        ctrl.n_costs = 2;
        
        % Specify additional costs if ctrl.type != 'ed guess'
        if ctrl.type ~= "ed guess" 
            if ctrl.estimate == "all"
                ctrl.n_costs = ctrl.n_costs + 4;
            else
                ctrl.n_costs = ctrl.n_costs + 1;
            end
        end
        
    end

    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    % Required for all controllers
    % ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ctrl.T_factor = sys.X_m/(sys.D*sys.pf);
    ctrl.U_ed = zeros(3*ctrl.N,1);

    % Generate the set U = {-1,0,1}^3
    u = [-1, 0, 1];
    [U1, U2, U3] = ndgrid(u, u, u);
    ctrl.U_set = [U1(:), U2(:), U3(:)]';
    
end