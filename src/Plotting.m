%% Setup

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
plot(t_sim_vec, T_sim(1,:), 'b');
plot(t_sim_vec, T_sim(2,:), 'r');
% plot(t_sim_vec, T_sim(3,:), 'g');
% plot(t_sim_vec, T_sim(3,:), 'm');
hold off;

title('Torque');
legend('Reference','No node limit', 'Node limit', ctrl2.type, ctrl3.type, 'Location','northwest');
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

% % Plot currents
% i_s = (sys.K_inv/(sys.X_s - sys.X_m/sys.X_r*sys.X_m)).*(psi_s_ctrl - sys.X_m/sys.X_r.*psi_r_ctrl);
% 
% figure(5);
% plot(t_ctrl_vec, i_s(:,1:n_controller_samples), 'b');
% 
% title('Currents');
% ylim([-1.2,1.2]);
% xlim([0,t_max]);
% grid on;

% Iterations (only useful to compare performance of different guesses)
ic = reshape([zeros(1,length(iter_count)); iter_count(1,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
ic2 = reshape([zeros(1,length(iter_count)); iter_count(2,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(iter_count));
ic3 = reshape([zeros(1,length(iter_count)); iter_count(3,:); zeros(1,length(iter_count))],1,3*length(iter_count));
tc3 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(iter_count));

figure(6);
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold on;
plot(tc,ic/10,'r');
plot(tc2 + ctrl0.T_s/6,ic2/10,'b');
plot(tc3 + ctrl0.T_s/3,ic3/10,'g');
hold off;

title('Iterations')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Iterations [10]')
legend('Torque reference','# Iterations opt','# Iterations ed','# Iterations sdp');
grid on;

% Nodes
nc0 = reshape([zeros(1,length(node_count)); node_count(1,:); zeros(1,length(node_count))],1,3*length(node_count));
nc1 = reshape([zeros(1,length(node_count)); node_count(2,:); zeros(1,length(node_count))],1,3*length(node_count));
nc2 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
nc3 = reshape([zeros(1,length(node_count)); node_count(4,:); zeros(1,length(node_count))],1,3*length(node_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(7);
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold on;
plot(tc,nc0/1000,'b');
plot(tc+ctrl0.T_s/6,nc1/1000,'r');
plot(tc+ctrl0.T_s*2/6, nc2/1000,'g');
plot(tc+ctrl0.T_s*3/6, nc3/1000,'m');
hold off;

title('Nodes')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('torque [pu] / # nodes [10^3]')
legend('Torque reference','opt', ctrl1.type, ctrl2.type, ctrl3.type, 'Location','northwest');
grid on;

% % Time
% nc = reshape([zeros(1,length(time_count)); time_count(1,:); zeros(1,length(time_count))],1,3*length(time_count));
% tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
% nc2 = reshape([zeros(1,length(time_count)); time_count(2,:); zeros(1,length(time_count))],1,3*length(time_count));
% tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
% nc3 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
% tc3 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
% 
% figure(8);
% plot(t_ctrl_vec, ref(1,1:end-1),'k');
% hold on;
% plot(tc,nc,'r');
% plot(tc2+ctrl0.T_s/6,nc2,'b');
% plot(tc3+ctrl0.T_s*2/6, nc3/100,'g');
% hold off;
% 
% title('Time')
% xlim([0,t_max]);
% xlabel('time [s]')
% ylabel('Torque [pu] / # Time [s]')
% legend('Torque reference','# Nodes no limit','# Nodes ed guess', '# Nodes ed guess + sdp');
% grid on;

% Accumulated Cost
J_acc_0 = cost_vec(1,:);
J_acc_1 = cost_vec(ctrl0.n_costs + 1,:);
J_acc_2 = cost_vec(ctrl0.n_costs+ctrl1.n_costs+1,:);
J_acc_3 = min(cost_vec(ctrl0.n_costs+ctrl1.n_costs+ctrl2.n_costs+...
    [1,3:ctrl2.n_costs],:));
for k = 2:length(J_acc_0)
    J_acc_0(k) = J_acc_0(k) + J_acc_0(k-1);
    J_acc_1(k) = J_acc_1(k) + J_acc_1(k-1);
    J_acc_2(k) = J_acc_2(k) + J_acc_2(k-1);
    J_acc_3(k) = J_acc_3(k) + J_acc_3(k-1);
end

figure(8);
plot(t_ctrl_vec, ref_ctrl,'k');
hold on;
plot(t_ctrl_vec,J_acc_0,'b');
plot(t_ctrl_vec,J_acc_1,'r');
plot(t_ctrl_vec,J_acc_2,'g');
plot(t_ctrl_vec,J_acc_3,'m');
hold off;

title('Accumulated Cost')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / Accumulated Cost')
legend('Torque reference','opt', ctrl1.type, ctrl2.type, ctrl3.type, 'Location','northwest');
% legend('Torque reference','Opt', 'Ed guess', 'Ed guess + sdp','Location','northwest');
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
del_u2 = abs([sim.u_0 squeeze(u_vec(:,3,1:end-1))] - squeeze(u_vec(:,3,:)));
f_sw2 = 1/(12*t_max)*sum(del_u2, 'all');
fprintf('Average switching frequency (ctrl0): %.2fHz \n', f_sw0);
fprintf('Average switching frequency (ctrl1): %.2fHz \n', f_sw1);
fprintf('Average switching frequency (ctrl2): %.2fHz \n\n', f_sw2);

% Print torque rms error
fprintf('Torque rms error: (ctrl0)            %.4fe-3 \n', ...
    norm(T_sim(1,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf('Torque rms error: (ctrl1)            %.4fe-3 \n', ...
    norm(T_sim(2,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf('Torque rms error: (ctrl2)            %.4fe-3 \n\n', ...
    norm(T_sim(3,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);

% Print absolute flux rms error
fprintf('Absolute flux rms error: (ctrl0)     %.4fe-3 \n', ...
    norm(Psi_sim(1,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf('Absolute flux rms error: (ctrl1)     %.4fe-3 \n', ...
    norm(Psi_sim(2,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf('Absolute flux rms error: (ctrl2)     %.4fe-3 \n\n', ...
    norm(Psi_sim(3,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);

% Print accumulated cost
fprintf('Average tracking cost: (ctrl0)       %.4fe-3 \n', ...
    J_acc_0(end)/length(J_acc_0)*1e3);
fprintf('Average tracking cost: (ctrl1)       %.4fe-3 \n', ...
    J_acc_1(end)/length(J_acc_1)*1e3);
fprintf('Average tracking cost: (ctrl2)       %.4fe-3 \n', ...
    J_acc_2(end)/length(J_acc_2)*1e3);



