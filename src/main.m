clear;

%% Setup
% Create empty structs to avoid errors
sys = struct();
sim = struct();
ctrl0 = struct();
ctrl1 = struct();
ctrl2 = struct();
ctrl3 = struct();

% -------------------------------------------------------------------------
% Quick setup of simulation parameters (set and remove whatever you want)
% -------------------------------------------------------------------------
ctrl0.node_limit = inf;
ctrl2.type = 'ed & sdp guess';
ctrl3.verbose = 1;
ctrl3.type = 'ed guess + sdp';
% -------------------------------------------------------------------------

% Physical system
sys = SystemSetup(sys);

% Simulation
sim = SimulationSetup(sys, sim, 'exact');

% Controllers
[ctrl0, run_ctrl0] = ControllerSetup(sys, 'n-step-SDP', ctrl0); % controller without node limit, gives J_opt as reference
[ctrl1, run_ctrl1] = ControllerSetup(sys, 'n-step-SDP', ctrl1); % controller with only ed guess
[ctrl2, run_ctrl2] = ControllerSetup(sys, 'n-step-SDP', ctrl2); % controller with ed and sdp guess
[ctrl3, run_ctrl3] = ControllerSetup(sys, 'n-step-SDP', ctrl3); % controller with ed guess and sdp
n_costs  = ctrl0.n_costs + ctrl1.n_costs + ctrl2.n_costs + ctrl3.n_costs; % required length of cost term
n_c = 4; % number of controllers that are simulated


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
cost_vec = NaN(n_costs, n_controller_samples);

% Sampled with simulation sampling time
x_vec_sim = nan*ones(4, n_c, n_simulation_samples);


%% Physical system simulation
% Initial parameters
x = repmat(sim.x_0,1,n_c);
u_prev = repmat(sim.u_0,1,n_c);

x_vec_sim(:,:,1) = x;
u_vec(:,:,1) = u_prev;

% Simulate
t_sim = tic;
for k = 1:n_controller_samples
    
    % Apply noise
    y = x + rand*(sim.b-sim.a)+sim.a;
    
    % Apply controller
    [u0, ctrl0, iter0, nodes0, times0, cost0] = run_ctrl0(y(:,1), u_prev(:,1), ref(:,k+1:end), ctrl0);
    [u1, ctrl1, iter1, nodes1, times1, cost1] = run_ctrl1(y(:,2), u_prev(:,2), ref(:,k+1:end), ctrl1);
    [u2, ctrl2, iter2, nodes2, times2, cost2] = run_ctrl2(y(:,3), u_prev(:,3), ref(:,k+1:end), ctrl2);
    [u3, ctrl3, iter3, nodes3, times3, cost3] = run_ctrl2(y(:,3), u_prev(:,3), ref(:,k+1:end), ctrl3);
    u = [u0 u1 u2 u3];
    
    % Apply physical system steps
    for j = 1:simulation_samples_per_controller_sample
        x = A_sim*x + B_sim*u;
        x_vec_sim(:,:,simulation_samples_per_controller_sample*(k-1)+j+1) = x;
    end
    
    % Update parameters for plotting
    u_vec(:,:,k) = u;
    iter_count(:,k) = [iter0; iter1; iter2; iter3];
    node_count(:,k) = [nodes0; nodes1; nodes2; nodes3];
    time_count(:,k) = [times0; times1; times2; times3];
    cost_vec(:,k) = [cost0; cost1; cost2; cost3]; % J_opt0, J_ed0, J_bnb1, J_ed1, J_bnb2, J_ed2, J_sdp2
    
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
Plotting


