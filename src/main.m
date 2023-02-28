% clear all; %clc; %close all;


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
steps = {[1 1 .1], [1 2 1]};
ramps = {[1 100e-6 0 .1 1], [1 3 1 5 0]};

% Physical system
sys = systemSetup(sys);

% Simulation
sim = simulationSetup(sys, sim, 'exact');

% Controller
[ctrl, run_ctrl] = controllerSetup(sys, '1-step', ctrl);


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
ref = generateReference(steps, ramps, n_controller_samples, int32(1/ctrl.T_s));

%% Variables for plotting
% Sampled with controller sampling time
u_vec = nan*ones(3, n_controller_samples + 1);
iter_count = nan*ones(1, n_controller_samples + 1);

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
    
    % Add zero-mean measurement noise
    y = x + normrnd(0,sys.std^2,length(x),1);
    
    % Apply controller
    [u, ctrl, iter] = run_ctrl(y, u_prev, ref(:,k:end), ctrl);
    
    % Apply physical system steps
    for j = 1:simulation_samples_per_controller_sample
        x = A_sim*x + B_sim*u;
        x_vec_sim(:,simulation_samples_per_controller_sample*(k-1)+j+1) = x;
    end
    
    % Update parameters for plotting
    u_vec(:,k+1) = u;
    iter_count(k+1) = iter;
    
    u_prev = u;
    
end
t_sim = toc(t_sim);

% Set first iteration count to the simulated mean value
iter_count(1) = mean(iter_count(2:end));


%% Plotting
Plotting


