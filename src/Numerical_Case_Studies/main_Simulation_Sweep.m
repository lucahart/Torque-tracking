clear;

% Add paths for used functions
addpath('./Controllers/')
addpath('./Numerical_Case_Studies/')
addpath('./Setup/')
addpath('./Tools/')
addpath(genpath('./YALMIP/'))


%% Setup
sys_pre = struct();
sim_pre = struct();

% -------------------------------------------------------------------------
% Setup simulation parameters (Can Optionally be changed)
% -------------------------------------------------------------------------
% We will run n_sims simulations with n_steps steps as specified:
n_sims = 3;
n_steps = 10;

% Step lengths are sampled from a uniform distribution on this interval:
step_interval = [.5, 1.5]; % in [pu]

% Initial torque value
torque_value_init = 0;

% Set random seed (will later be overwritten by single sims) 
random_seed = 42;
rng(random_seed);

% -------------------------------------------------------------------------

% Create only the physical system
sys = SystemSetup(sys_pre);

% The simulation length must be specified
sim_pre.n_fundamentals = 10;

%
step_time_min = .3;

% The controller sampling time must be specified
ctrl_pre.T_s = 25e-6;

% Simulation sweep
% sim_pre.ramps = repmat({{}},1,n_sims);
steps_pre = repmat({{}},1,n_sims);

% Set the measurement length for computing the error after each step
measurement_length = 5e-3;


%% Precalculations

% Simulation length
sim_length = sim_pre.n_fundamentals/(sys.f_r*sys.f_base);

% Store steps in appropriate form
for sim_cnt = 1:n_sims
    torque_values = bounded_random_walk(torque_value_init, step_interval(1), step_interval(2), n_steps);
    step_times = step_time_walk(step_time_min, n_steps, sim_length);
    steps_pre{sim_cnt}{1} = {[1, 1, torque_value_init]};
    for step_cnt = 1:n_steps
        steps_pre{sim_cnt}{step_cnt+1} = {[1, ...
            round(step_times(step_cnt)/ctrl_pre.T_s), ...
            double(torque_values(step_cnt+1))]};
    end
end


%% Variables to store all simulation results
u_sims = cell(n_sims,1);
iter_count_sims = cell(n_sims,1);
node_count_sims = cell(n_sims,1);
time_count_sims = cell(n_sims,1);
cost_vec_sims = cell(n_sims,1);
x_vec_sims = cell(n_sims,1);
step_times_sims = cell(n_sims,1);
ref_sims = cell(n_sims,1);


%% Run simulations
for sim_cnt = 1:n_sims

    sim_pre.ramps = {};
    sim_pre.steps = cellfun(@(c) c{1}, steps_pre{sim_cnt}, 'UniformOutput', false);
    
    
    %% Run single simulation
    
    Single_Simulation
    
    % Store simulation results 
    u_sims{sim_cnt} = u_vec;
    iter_count_sims{sim_cnt} = iter_count;
    node_count_sims{sim_cnt} = node_count;
    time_count_sims{sim_cnt} = time_count;
    cost_vec_sims{sim_cnt} = cost_vec;
    x_vec_sims{sim_cnt} = x_vec_sim;
    step_times_sims{sim_cnt} = step_times;
    ref_sims{sim_cnt} = ref;

end

%% Plotting
show_simulation = 3;

u_vec = u_sims{show_simulation};
iter_count = iter_count_sims{show_simulation};
node_count = node_count_sims{show_simulation};
time_count = time_count_sims{show_simulation};
cost_vec = cost_vec_sims{show_simulation};
x_vec_sim = x_vec_sims{show_simulation};
steps = cellfun(@(c) c{1}, steps_pre{show_simulation}, 'UniformOutput', false);
ref = ref_sims{show_simulation};

Plotting

PlottingAllSims
