%% Setup
% Display parameters
display = displaySetup;

% Some parameters for easier plotting and printing
% --- Time parameters -----------------------------------------------------
t_ctrl_vec = double(1:n_controller_samples+1)*ctrl.T_s;
t_sim_vec = double(1:n_simulation_samples+1)*sim.T_sim;
t_max = double(n_controller_samples*ctrl.T_s);
% --- Simulation parameters -----------------------------------------------
psi_s_sim = x_vec_sim(1:2,:);
psi_r_sim = x_vec_sim(3:4,:);
% --- Controller parameters -----------------------------------------------
x_vec_ctrl = x_vec_sim(:,1:simulation_samples_per_controller_sample:end);
psi_s_ctrl = x_vec_ctrl(1:2,:);
psi_r_ctrl = x_vec_ctrl(3:4,:);


%% Plotting
% Plot states
figure(1);
plot(t_ctrl_vec,x_vec_ctrl);

title('Fluxes');
legend('\psi_{s,\alpha}','\psi_{s,\beta}','\psi_{r,\alpha}','\psi_{r,\beta}');
xlim([0,t_max]);
grid on;

% Plot inputs
figure(2)

subplot(3,1,1);
plot(t_ctrl_vec,u_vec(1,:))
subplot(3,1,2);
plot(t_ctrl_vec,u_vec(2,:))
subplot(3,1,3);
plot(t_ctrl_vec,u_vec(3,:))

subplot(3,1,1);
title('Inputs');
ylim([-1.2,1.2]);
xlim([0,t_max]);
grid on;
subplot(3,1,2);
ylim([-1.2,1.2]);
xlim([0,t_max]);
grid on;
subplot(3,1,3);
ylim([-1.2,1.2]);
xlim([0,t_max]);
grid on;

% Plot torque
T_sim = 1/sys.pf*sys.X_m/sys.D*(psi_r_ctrl(1,:).*psi_s_ctrl(2,:) - psi_r_ctrl(2,:).*psi_s_ctrl(1,:));

figure(3);
plot(t_ctrl_vec, T_sim);

title('Torque');
ylim([0,1.2]);
xlim([0,t_max]);
grid on;

% Plot stator flux
Psi_sim = vecnorm(psi_s_ctrl,2,1);

figure(4);
plot(t_ctrl_vec, Psi_sim);

title('Absolute stator flux');
ylim([0,1.2]);
xlim([0,t_max]);
grid on;

% Plot currents
i_s = sys.K_inv*((sys.X_s - sys.X_m/sys.X_r*sys.X_m)\(psi_s_ctrl - sys.X_m/sys.X_r*psi_r_ctrl));

figure(5);
plot(t_ctrl_vec, i_s(:,1:n_controller_samples+1), 'b');

title('Currents');
ylim([-1.2,1.2]);
xlim([0,t_max]);
grid on;

%% Printing
% 
fprintf('---------------------------------------------\n');

% Print elapsed simulation time
fprintf('Elapsed simulation time:     %.2fs \n', t_sim);

% Print average switching frequency
del_u = abs([sim.u_0 u_vec(:,1:length(u_vec)-1)] - u_vec);
f_sw = 1/(12*t_max)*sum(del_u, 'all');
fprintf('Average switching frequency: %.2fHz \n', f_sw);


% -------------------------------------------------------------------------
T_sim = 1/sys.pf*sys.X_m/sys.D*(psi_r_sim(1,:).*psi_s_sim(2,:) - psi_r_sim(2,:).*psi_s_sim(1,:));
Psi_sim = vecnorm(psi_s_sim,2,1);
f = @(r,n) [r(1); reshape([r' r'.*ones(length(r), n-1)]', length(r)*(n),1)];
T_ref_sim = f(ref(1,:),simulation_samples_per_controller_sample)';
Psi_ref_sim = f(ref(2,:), simulation_samples_per_controller_sample)';
T_ctrl = 1/sys.pf*sys.X_m/sys.D*(psi_r_ctrl(1,:).*psi_s_ctrl(2,:) - psi_r_ctrl(2,:).*psi_s_ctrl(1,:));
Psi_ctrl = vecnorm(psi_s_ctrl,2,1);
T_ref_ctrl = [ref(1,1), ref(1,:)];
Psi_ref_ctrl = [ref(2,1), ref(2,:)];
J = 1/double(n_controller_samples)*sum(ctrl.lam_T*(T_ctrl-T_ref_ctrl).^2 + (1-ctrl.lam_T)*(Psi_ctrl-Psi_ref_ctrl).^2);% + ctrl.lam_u*norm(del_u,1));
% -------------------------------------------------------------------------


% Print torque rms error
fprintf('Torque rms error:            %.4fe-3 \n', norm(T_sim-T_ref_sim,2)/sqrt(length(T_sim))*1e3);

% Print absolute flux rms error
fprintf('Absolute flux rms error:     %.4fe-3 \n', norm(Psi_sim-Psi_ref_sim,2)/sqrt(length(T_sim))*1e3);

% Print accumulated cost
fprintf('Accumulated tracking cost:   %.4fe-3 \n', J*1e3);

% Print average sphere decoding nodes used
fprintf('Mean sphere decoding nodes:  %0.4f \n',mean(iter_count))

% 
results(x_vec_sim, u_vec, iter_count, sim.T_sim, controller_samples_per_round*simulation_samples_per_controller_sample,controller_samples_per_round, 0, 1, sys.K_inv, sys);



