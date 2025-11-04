clear;

% Check if SCS is available
if (exist('scs','file') ~= 2)
    error("SCS could not be found.");
end

% Add paths
addpath('./Controllers/')
addpath('./Numerical_Case_Studies/')
addpath('./Setup/')
addpath('./Plotting/')
addpath('./Tools/')
addpath(genpath('./YALMIP/'))


%% Setup
% Create empty structs to avoid errors
sys = struct();
sim = struct();
ctrl0 = struct();
ctrl1 = struct();
ctrl2 = struct();

% -------------------------------------------------------------------------
% Quick setup of simulation parameters (set and remove whatever you want)
% -------------------------------------------------------------------------
ctrl0.node_limit = inf;
ctrl1.node_limit = inf;
ctrl2.node_limit = inf;
% -------------------------------------------------------------------------

% Physical system
sys = SystemSetup(sys);

% Simulation
sim = SimulationSetup(sys, sim, 'exact');

% Controllers
[ctrl0, run_ctrl0] = ControllerSetup(sys, 'n-step-SDP', ctrl0); % controller initialized with ed guess
[ctrl1, run_ctrl1] = ControllerSetup(sys, 'n-step-SDP', ctrl1); % controller initialized with opt guess
[ctrl2, run_ctrl2] = ControllerSetup(sys, 'n-step-SDP', ctrl2); % controller initialized with bad guess
n_costs  = ctrl0.n_costs + ctrl1.n_costs + ctrl2.n_costs; % required length of cost term
n_c = 3; % number of controllers that are simulated


%% Precalculations for faster simulation
controller_samples_per_round = int32(1/(ctrl0.T_s*sys.f_r*sys.f_base));
simulation_samples_per_controller_sample = int32(ctrl0.T_s/sim.T_sim);
n_controller_samples = controller_samples_per_round*sim.n_fundamentals;
n_simulation_samples = n_controller_samples*simulation_samples_per_controller_sample;
% Simulation dynamics
A_sim = [
    sim.A_1 sim.B_1;
    sim.B_3 sim.A_2;
];
B_sim = [
    sim.B_2;
    sim.B_4;
];
% Reference
ref = generate_reference(sim.steps, sim.ramps, n_controller_samples, ...
    int32(1/(ctrl0.T_s*sys.f_r*sys.f_base)));

%% Variables for plotting
% Sampled with controller sampling time
u_vec = NaN(3, n_c, n_controller_samples);
iter_count = NaN(n_c, n_controller_samples);
node_count = NaN(n_c, n_controller_samples);
time_count = NaN(n_c, n_controller_samples);
cost_vec = NaN(n_costs, n_controller_samples); %size dependent on # and ctrl type

% Sampled with simulation sampling time
x_vec_sim = nan*ones(4, n_c, n_simulation_samples);


%% Physical system simulation
% Initial parameters
x = repmat(sim.x_0,1,n_c);
u_prev = repmat(sim.u_0,1,n_c);

% x_vec_ctrl(:,1) = x;
x_vec_sim(:,:,1) = x;
u_vec(:,:,1) = u_prev;

% Simulate
t_sim = tic;
for k = 1:n_controller_samples
    
    % Apply noise
    y = x + rand*(sim.b-sim.a)+sim.a;
    
    % Apply controller
    [u0, ctrl0, iter0, nodes0, times0, cost0] = run_ctrl0(y(:,1), u_prev(:,1), ref(:,k+1:end), ctrl0);
    ctrl1.U_ed = [u0; ctrl0.U_ed(1:end-3)]; % assign optimal solution as initial solution of ctrler 1
    [u1, ctrl1, iter1, nodes1, times1, cost1] = run_ctrl1(y(:,2), u_prev(:,2), ref(:,k+1:end), ctrl1);
    ctrl2.U_ed = zeros(ctrl2.N*3,1); % assign a bad initial guess to ctrler 2
    [u2, ctrl2, iter2, nodes2, times2, cost2] = run_ctrl2(y(:,3), u_prev(:,3), ref(:,k+1:end), ctrl2);
    u = [u0 u1 u2];
    
    % Apply physical system steps
    for j = 1:simulation_samples_per_controller_sample
        x = A_sim*x + B_sim*u;
        x_vec_sim(:,:,simulation_samples_per_controller_sample*(k-1)+j+1) = x;
    end
    
    % Update parameters for plotting
    u_vec(:,:,k) = u;
    iter_count(:,k) = [iter0; iter1; iter2];
    node_count(:,k) = [nodes0; nodes1; nodes2];
    time_count(:,k) = [times0; times1; times2];
    cost_vec(:,k) = [cost0; cost1; cost2]; % J_opt0, J_ed0, J_opt1, J_ed1, J_opt2, J_ed2
    
    % Update u_prev
    u_prev = u;
    
    % If only opt solution is applied the states for the controller are
    % reset to the optimal dynamics every controller sample
    if sim.apply_only_opt
        x = x(:,1).*ones(4,n_c);
        u_prev = u_prev(:,1).*ones(3,n_c);
    end
end
t_sim = toc(t_sim);

% Remove last state to keep the same length as all other vectors
x_vec_sim(:,:,end) = [];


%% Plotting
Plotting_Initial_Guess


