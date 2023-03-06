function sim = simulationSetup(sys, sim_pre, discretization)
    % Setup of all simulation parameters. 
    % Returns a struct sim.
    
    
    % *********************************************************************
    %% Make changes to this section (simulation parameters)
    % *********************************************************************
    
    % Simulation
    sim.n_fundamentals = 10; % number of fundamentals to simulate
    sim.T_sim = .5e-6; % simulation sampling interval [s], please choose 
                       % such that ctrl.T_s = n*sim.T_sim, n is integer
    
    % Initial state and input
    sim.x_0 = [-.99 -.15 -.91 .07]'; % Initialization to steady-state
    sim.u_0 = zeros(3,1);
    
    
    % *********************************************************************
    %% Do NOT make changes to this section
    % *********************************************************************
    
    % Use pre-specified simulation parameters if given
    % Overwrites the default setup of the section above
    if nargin >= 2
        fn = fieldnames(sim_pre);
        for i = 1:numel(fn)
            sim.(fn{i}) = sim_pre.(fn{i});
        end
    end
    
    % Calculate discrete-time matrices
    % Specification of either exact euler or forward euler discretization
    % is possible. Default is exact discretization.
    if nargin <= 2 || discretization == "exact"
        sim.A_1 = expm(sys.F_1*sim.T_sim*sys.w_base);
        sim.A_2 = expm(sys.F_2*sim.T_sim*sys.w_base);
        sim.B_1 = sys.F_1\(sim.A_1-eye(length(sim.A_1)))*sys.G_1;
        sim.B_2 = sys.F_1\(sim.A_1-eye(length(sim.A_1)))*sys.G_2;
        sim.B_3 = sys.F_2\(sim.A_2-eye(length(sim.A_2)))*sys.G_3;
        sim.B_4 = sys.F_2\(sim.A_2-eye(length(sim.A_2)))*sys.G_4;
    elseif discretization == "forward"
        sim.A_1 = eye(2) + sys.F_1*sim.T_sim*sys.w_base;
        sim.A_2 = eye(2) + sys.F_2*sim.T_sim*sys.w_base;
        sim.B_1 = sys.G_1*sim.T_sim*sys.w_base;
        sim.B_2 = sys.G_2*sim.T_sim*sys.w_base;
        sim.B_3 = sys.G_3*sim.T_sim*sys.w_base;
        sim.B_4 = sys.G_4*sim.T_sim*sys.w_base;
    else
        error('Discretization can be exact or forward');
    end
    
end