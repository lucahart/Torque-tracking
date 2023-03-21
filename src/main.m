% clear all;

%% Setup
% Create empty structs to avoid errors
sys = struct();
sim = struct();
ctrl = struct();
steps = {};
ramps = {};

% Quick setup of simulation parameters (set and remove whatever you want)
sim.n_fundamentals = 50;
sys.std = 0e-2;
steps = {[1 20 .5], [1 30 1]}; % step up @ k=16059, step down @ k=24088
ramps = {[1 100e-6 0 10 1], [1 40 1 50 0]};
execute_sdp = [1:50,16000:16100,24050:24350];

% Physical system
sys = systemSetup(sys);

% Simulation
sim = simulationSetup(sys, sim, 'exact');

% Controller
[ctrl, run_ctrl] = controllerSetup(sys, 'n-step-SDP', ctrl);


%% Precalculations for faster simulation
controller_samples_per_round = int32(1/(ctrl.T_s*sys.f_r*sys.f_base));
simulation_samples_per_controller_sample = int32(ctrl.T_s/sim.T_sim);
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
ref = generateReference(steps, ramps, n_controller_samples, ...
    int32(1/(ctrl.T_s*sys.f_r*sys.f_base)));
ex_sdp = zeros(1,n_controller_samples);
ex_sdp(execute_sdp) = 1;

%% Variables for plotting
% Sampled with controller sampling time
u_vec = NaN(3, n_controller_samples);
iter_count = NaN(2, n_controller_samples);
node_count = NaN(2, n_controller_samples);
time_count = NaN(2, n_controller_samples);
t_exec_ctrl_vec = NaN(1, n_controller_samples);
cost_vec = NaN(6, n_controller_samples);

% Sampled with simulation sampling time
x_vec_sim = nan*ones(4, n_simulation_samples);


%% Physical system simulation
% Initial parameters
x = sim.x_0;
u_prev = sim.u_0;

% x_vec_ctrl(:,1) = x;
x_vec_sim(:,1) = x;
u_vec(:,1) = u_prev;

% Simulate
t_sim = tic;
for k = 1:n_controller_samples    
    
    % Apply noise
    y = x + normrnd(0, sys.std);
    
    % Apply controller
    t_ctrl = tic;
    [u, ctrl, iter, nodes, times, cost] = run_ctrl(y, u_prev, ref(:,k+1:end), ctrl);
    t_ctrl = toc(t_ctrl);
    
    % Apply physical system steps
    for j = 1:simulation_samples_per_controller_sample
        x = A_sim*x + B_sim*u;
        x_vec_sim(:,simulation_samples_per_controller_sample*(k-1)+j+1) = x;
    end
    
    % Update parameters for plotting
    u_vec(:,k) = u;
    iter_count(:,k) = iter;
    node_count(:,k) = nodes;
    time_count(:,k) = times;
    t_exec_ctrl_vec(k) = t_ctrl;
    cost_vec(:,k) = cost;
    
    u_prev = u;
    
end
t_sim = toc(t_sim);


%% Plotting
Plotting


