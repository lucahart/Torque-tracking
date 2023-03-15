%% Setup
yalmip('clear');
opt = optimoptions('fmincon','Display','off');
epsilon = 1e-4;
sdpopt = sdpsettings('solver', 'scs', 'verbose', 0, 'dualize', 0, 'scs.eps_abs', epsilon, 'scs.eps_rel', epsilon);
prec = 1e-3;

n_x = 4;
n_x2 = 2;
m = 3;

%% Setup
% % Create empty structs to avoid errors
% sys = struct();
% sim = struct();
% ctrl = struct();
% steps = {};
% ramps = {};
% 
% % Quick setup of simulation parameters (set and remove whatever you want)
% sim.n_fundamentals = 50;
% sys.std = 0e-2;
% steps = {[1 20 .5], [1 30 1]};
% ramps = {[1 100e-6 0 10 1], [1 40 1 50 0]};
% 
% % Physical system
% sys = systemSetup(sys);
% 
% % Simulation
% sim = simulationSetup(sys, sim, 'exact');
% 
% % Controller
% [ctrl, run_ctrl] = controllerSetup(sys, 'n-step-SDP', ctrl);
% 
% 
% %% Precalculations for faster simulation
% controller_samples_per_round = int32(1/(ctrl.T_s*sys.f_r*sys.f_base));
% simulation_samples_per_controller_sample = int32(ctrl.T_s/sim.T_sim);
% n_controller_samples = controller_samples_per_round*sim.n_fundamentals;
% n_simulation_samples = n_controller_samples*simulation_samples_per_controller_sample;
% % Simulation dynamics
% A_sim = [
%     sim.A_1 sim.B_1;
%     sim.B_3 sim.A_2;
% ];
% B_sim = [
%     sim.B_2;
%     sim.B_4;
% ];
% % Reference
% ref = generateReference(steps, ramps, n_controller_samples, ...
%     int32(1/(ctrl.T_s*sys.f_r*sys.f_base)));

%% Values
N = ctrl.N;
A_sim = ctrl.A_sim;
B_sim = ctrl.B_sim;
% T_ref = 1;
% Psi_ref = 1;
% lb = -1;
% ub = 1;
% x = [1 -1 1 -1]';
% u_prev = [1 -1 0]';

%% Matrices
Gamma_s = zeros(2*N,n_x);
for i = 0:N-1
    Gamma_s(2*i + 1:2*(i+1), :) = A_sim(1:2,:)*A_sim^i;
end

Upsilon_s = zeros(2*N,3*N);
for i = 0:N-1
    for j = 0:i
        if i == j
            Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = ctrl.B_2;
        else
            Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A_sim(1:2,:)*A_sim^(i-j-1)*B_sim;
        end
    end
end

Gamma_r = zeros(2*N,n_x);
for i = 0:N-1
    Gamma_r(2*i + 1:2*(i+1), :) = A_sim(3:4,:)*A_sim^i;
end

Upsilon_r = zeros(2*N,3*N);
for i = 0:N-1
    for j = 0:i-1
        Upsilon_r(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A_sim(3:4,:)*A_sim^(i-j-1)*B_sim;
    end
end

Zeta = [0 1; -1 0];

% S = eye(3*N) - [
%     zeros(3,3*N);
%     eye(3*(N-1)) zeros(3*(N-1),3)
% ];
% S = [
%     zeros(1,3*N+1);
%     S, zeros(3*N,1);
% ];
% 
% E = [zeros(1,3); eye(3); zeros(3*(N-1),3)];

Q = @(x,i) [
    x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:);
    Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:)
];

W = @(x,i) [
    x'*Gamma_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, x'*Gamma_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:);
    Upsilon_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, Upsilon_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:)
];

Y0a = [-u_prev(1) 1 zeros(1,3*N-1); zeros(3*N,3*N+1)];
Y0b = [-u_prev(2) 0 1 zeros(1,3*N-2); zeros(3*N,3*N+1)];
Y0c = [-u_prev(3) 0 0 1 zeros(1,3*N-3); zeros(3*N,3*N+1)];

%% Optimization
X = sdpvar(3*N+1);

% Constraint on PSD matrix and constant multiplier to 1
F = [
    X >= 0,
    X(1) == 1,
];
% Constraint on diagonal terms
F = [F, 0 <= diag(X) <= 1];
% Constraint on off-diagonal terms (only holds for lb=-1 and ub=1)
F = [F, -1 <= X(logical(triu(true(3*N+1))-diag(true(3*N+1,1)))) <= 1];

% Cost function
J = 0;
% Add switching penalty to cost term
J = J + ctrl.lam_u*norm(trace(Y0a*X),1);
J = J + ctrl.lam_u*norm(trace(Y0b*X),1);
J = J + ctrl.lam_u*norm(trace(Y0c*X),1);
for i = 2:3*N-2
    Y = [zeros(1,i-1), -1, zeros(1,2), 1, zeros(1, 3*N+1-(i+3)); zeros(3*N,3*N+1)];
    J = J + ctrl.lam_u*norm(trace(Y*X),1);
end
% Add tracking penalties to cost term
D = [];
for i = 0:N-1
    D = [
        D; 
        sqrt(ctrl.lam_T)*(T_ref - ctrl.T_factor*trace(Q(x,i)*X));
        sqrt(1-ctrl.lam_T)*(Psi_ref^2 - trace(W(x,i)*X))
    ];
end
J = J + norm(D,2)^2;
% for i = 0:N-1
%     J = J + ctrl.lam_T*norm(T_ref - ctrl.T_factor*trace(Q(x,i)*X),2) ...
%         + (1-ctrl.lam_T)*norm(Psi_ref^2 - trace(W(x,i)*X),2);
% end
% Perform optimization
% assign(X,[1; ctrl.U_ed]*[1 ctrl.U_ed']);
t2 = tic;
optimize(F, J, sdpopt);
t2 = toc(t2);
% Calculate approximated value
% y_app = value(norm(T_ref-ctrl.T_factor*trace(Q(x)*X),2));
[V, Lambda] = eig(value(X));
v1 = V(:,end)*sqrt(Lambda(end));
v1 = sign(v1(1))*v1(2:end);
% round normally
u_hat1 = round(v1);
% round with equal distribution of all numbers
u_hat2 = zeros(3*N,1);
u_hat2(v1 > 1/3) = 1;
u_hat2(v1 < -1/3) = -1;


