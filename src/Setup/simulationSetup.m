function sim = SimulationSetup(sys, sim_pre, discretization)
    % SIMULATIONSETUP sets up a struct containing all inforamtion about the
    % simulation.
    %
    % Input:
    %   sys: System that is to be simulated.
    %   sim_pre (optional): Pre-set simulation parameters. Everything that
    %     is set in this struct will overwrite the default parameters. All
    %     possible parameters and their default values are listed below.
    % Output:
    %   sim: Struct containing all information about the simulation.
    %
    % Parameters:
    %   sim.n_fundamentals: Number of fundamentals to simulate.
    %     Default: n_fundamentals = 10
    %   sim.T_sim: Simulation sampling interval [s]. Choose such that
    %     ctrl.T_s = n*sim.T_sim, where n is a positive integer.
    %     Default: T_sim = .5e-6
    %   sim.apply_only_opt: If apply_only_opt = 1, then the simulation 
    %     executes all controllers, but sets the states x and previous 
    %     inputs u_prev to the value of only the first controller 
    %     (see main). Can be useful for making a fairer comparison.
    %     If apply_only_opt = 0, the all controllers are executed and the 
    %     inputs are applied to seperate systems paralelly.
    %     Default: apply_only_opt = 0
    %   sim.steps: Cell-object containing all information about the steps
    %     that are applied to the reference. See generate_reference for a
    %     detailled documentation of the used format.
    %     Default: steps = {[1 3 .2], [1 7 1]}
    %   sim.ramps: Cell-object containing all information about the ramps
    %     that are applied to the reference. See generate_reference for a
    %     detailled documentation of the used format.
    %     Default: ramps = {[1 100e-6 0 2 1], [1 8 1 10 0]}
    %   sim.x_0: Initial state of the system.
    %     Default: x_0 = [-.99 -.15 -.91 .07]' (steady-state, Psi=1, T=.5)
    %   sim.u_0: Initial input of the system.
    %     Default: u_0 = [0 0 0]'
    
    
    % *********************************************************************
    %% Default simulation parameters (Make changes to this section)
    % *********************************************************************
    
    % Simulation
    sim.n_fundamentals = 10;
    sim.T_sim = .5e-6;
    sim.apply_only_opt = 0;
    sim.steps = {[1 3 .2], [1 7 1]};
    sim.ramps = {[1 100e-6 0 2 1], [1 8 1 10 0]};
    
    % Initial state and input (steady-state values)
    sim.x_0 = [-.99 -.15 -.91 .07]';
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