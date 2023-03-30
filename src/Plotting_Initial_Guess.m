% Some parameters for easier plotting and printing
% --- Time parameters -----------------------------------------------------
t_ctrl_vec = double(0:n_controller_samples-1)*ctrl0.T_s;
t_sim_vec = double(0:n_simulation_samples-1)*sim.T_sim;
t_max = double(n_controller_samples-1)*ctrl0.T_s;
% --- Simulation parameters -----------------------------------------------
psi_s_sim = x_vec_sim(1:2,:,:);
psi_r_sim = x_vec_sim(3:4,:,:);
ref_sim = interp1([t_ctrl_vec double(n_controller_samples)*ctrl0.T_s], ref(:,:)', [t_sim_vec double(n_simulation_samples)*sim.T_sim])';
ref_sim = ref_sim(:,1:end-1);
% --- Controller parameters -----------------------------------------------
x_vec_ctrl = x_vec_sim(:,:,1:simulation_samples_per_controller_sample:end);
psi_s_ctrl = x_vec_ctrl(1:2,:,:);
psi_r_ctrl = x_vec_ctrl(3:4,:,:);
ref_ctrl = ref(1,1:end-1);

if 1
%% Plotting
% Plot states
figure(1);

ctrler = 1; % change 1 to 2 or 3 to get respective controlled dynamics
plot(t_sim_vec,squeeze(x_vec_sim(:,ctrler,:)));

title('Optimal Fluxes');
legend('\psi_{s,\alpha}','\psi_{s,\beta}','\psi_{r,\alpha}','\psi_{r,\beta}');
xlim([0,t_max]);
grid on;

% Plot inputs
figure(2)

ctrler = 1; % change 1 to 2 or 3 to get respective control inputs
subplot(3,1,1);
plot(t_ctrl_vec,squeeze(u_vec(1,ctrler,:))) 
subplot(3,1,2);
plot(t_ctrl_vec,squeeze(u_vec(2,ctrler,:)))
subplot(3,1,3);
plot(t_ctrl_vec,squeeze(u_vec(3,ctrler,:)))

subplot(3,1,1);
title('Optimal Inputs');
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
T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));

figure(3);
plot(t_sim_vec, ref_sim(1,:), 'k')
hold on;
plot(t_sim_vec, T_sim);
hold off;

title('Torque');
legend('Reference', 'Ed guess', 'Opt guess');
ylim([0,1.2]);
xlim([0,t_max]);
xlabel('time [s]');
ylabel('torque [pu]');
grid on;

% Plot stator flux
Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));

figure(4);
plot(t_sim_vec, Psi_sim);
hold on;
plot(t_sim_vec, ref_sim(2,:), 'k');
hold off;

title('Absolute stator flux');
ylim([0,1.2]);
xlim([0,t_max]);
grid on;

% Iterations (only useful to compare performance of different guesses)
ic = reshape([zeros(1,length(iter_count)); iter_count(1,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
ic1 = reshape([zeros(1,length(iter_count)); iter_count(2,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc1 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(iter_count));
ic2 = reshape([zeros(1,length(iter_count)); iter_count(3,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(iter_count));

figure(6);
plot(tc2 + ctrl0.T_s/3,ic2/10,'g');
hold on;
plot(tc,ic/10,'r');
plot(t_ctrl_vec, ref(1,1:end-1),'k');
plot(tc1 + ctrl0.T_s/6,ic1/10,'b');
hold off;

title('Iterations')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Iterations [10]')
legend('Torque reference', 'Initial solution (1)', 'Initial Solution (2)', 'Initial Solution (3)');
grid on;

% Nodes
nc = reshape([zeros(1,length(node_count)); node_count(1,:); zeros(1,length(node_count))],1,3*length(node_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc1 = reshape([zeros(1,length(node_count)); node_count(2,:); zeros(1,length(node_count))],1,3*length(node_count));
tc1 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc2 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(7);
plot(tc2+ctrl0.T_s/3,nc2/1e4,'g');
hold on;
plot(tc,nc/1e4,'r');
plot(tc1+ctrl0.T_s/6,nc1/1e4,'b');
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold off;

title('Nodes')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Nodes [1e4]')
legend('Torque reference', 'Initial solution (1)', 'Initial Solution (2)', 'Initial Solution (3)');
grid on;

% Time
nc = reshape([zeros(1,length(time_count)); time_count(1,:); zeros(1,length(time_count))],1,3*length(time_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
J_acc_2 = reshape([zeros(1,length(time_count)); time_count(2,:); zeros(1,length(time_count))],1,3*length(time_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(8);
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold on;
plot(tc,nc,'r');
plot(tc2+ctrl0.T_s/6,J_acc_2,'b');
hold off;

title('Time')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Time [s]')
legend('Torque reference', 'Ed guess', 'Opt guess');
grid on;
end

%% Printing
% 
fprintf('\n--------------------------------------------------------\n');

% Print elapsed simulation time
fprintf('Elapsed simulation time:     %.2fs \n\n', t_sim);

% Print average switching frequency
del_u0 = abs([sim.u_0 squeeze(u_vec(:,1,1:end-1))] - squeeze(u_vec(:,1,:)));
f_sw0 = 1/(12*t_max)*sum(del_u0, 'all');
del_u1 = abs([sim.u_0 squeeze(u_vec(:,2,1:end-1))] - squeeze(u_vec(:,2,:)));
f_sw1 = 1/(12*t_max)*sum(del_u1, 'all');
fprintf('Average switching frequency (Opt):            %.2fHz \n', f_sw0);
fprintf('Average switching frequency (Ed guess):       %.2fHz \n', f_sw1);
% Print torque rms error
fprintf('Torque rms error: (Opt)                       %.4fe-3 \n', ...
    norm(T_sim(1,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf('Torque rms error: (Ed guess)                  %.4fe-3 \n', ...
    norm(T_sim(2,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);

% Print absolute flux rms error
fprintf('Absolute flux rms error: (Opt)                %.4fe-3 \n', ...
    norm(Psi_sim(1,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf('Absolute flux rms error: (Ed guess)           %.4fe-3 \n', ...
    norm(Psi_sim(2,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);

% % Print accumulated cost
% fprintf('Accumulated tracking cost: (Opt)              %.4fe-3 \n', ...
%     J_acc_0(end)/length(J_acc_0)*1e3);
% fprintf('Accumulated tracking cost: (Ed guess)         %.4fe-3 \n', ...
%     J_acc_1(end)/length(J_acc_1)*1e3);


