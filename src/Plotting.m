% *************************************************************************
%% Setup
% *************************************************************************

% Some parameters for easier plotting and printing
% --- Time parameters -----------------------------------------------------
t_ctrl_vec = double(0:n_controller_samples-1)*ctrl0.T_s;
t_sim_vec = double(0:n_simulation_samples-1)*sim.T_sim;
t_max = double(n_controller_samples-1)*ctrl0.T_s;
% --- Simulation parameters -----------------------------------------------
psi_s_sim = x_vec_sim(1:2,:,:);
psi_r_sim = x_vec_sim(3:4,:,:);
ref_sim = interp1([t_ctrl_vec double(n_controller_samples)*ctrl0.T_s], ...
    ref(:,:)', [t_sim_vec double(n_simulation_samples)*sim.T_sim])';
ref_sim = ref_sim(:,1:end-1);
% --- Controller parameters -----------------------------------------------
x_vec_ctrl = x_vec_sim(:,:,1:simulation_samples_per_controller_sample:end);
psi_s_ctrl = x_vec_ctrl(1:2,:,:);
psi_r_ctrl = x_vec_ctrl(3:4,:,:);
ref_ctrl = ref(1,1:end-1);

% *************************************************************************
%% Plotting
% *************************************************************************

if 0 % comment out some plots

% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot states
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ctrler = 1; % change 1 to 2 or 3 to get respective controlled dynamics

figure(1);
plot(t_sim_vec,squeeze(x_vec_sim(:,ctrler,:)));

title('Optimal Fluxes');
legend('\psi_{s,\alpha}','\psi_{s,\beta}','\psi_{r,\alpha}','\psi_{r,\beta}');
xlim([0,t_max]);
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot inputs
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ctrler = 1; % change 1 to 2 or 3 to get respective control inputs

figure(2);
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

end % end comment out of plots


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot torque
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));

figure(3);
plot(t_sim_vec, ref_sim(1,:), 'k')
hold on;
plot(t_sim_vec, T_sim(1,:), 'b');
plot(t_sim_vec, T_sim(2,:), 'r');
% plot(t_sim_vec, T_sim(3,:), 'm');
plot(t_sim_vec, T_sim(4,:), 'g');
hold off;

title('Torque');
legend('reference', 'opt', 'limit', 'sdp', 'Location','southwest');%'opt', ctrl1.type, ctrl2.type, ctrl3.type, 'Location','southwest');
ylim([-1.3,1.3]); % ylim([-1,1.3]);
xlim([0,t_max]); % xlim([.65,.75]);
xlabel('time [s]');
ylabel('torque [pu]');
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot stator flux
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));

figure(4);
plot(t_sim_vec, ref_sim(2,:), 'k');
hold on;
plot(t_sim_vec, Psi_sim(1,:), 'b');
plot(t_sim_vec, Psi_sim(2,:), 'r');
% plot(t_sim_vec, Psi_sim(3,:), 'm');
plot(t_sim_vec, Psi_sim(4,:), 'g');
hold off;

title('Absolute stator flux');
legend('Flux reference','opt', ctrl1.type, ctrl3.type, 'Location','southwest');
ylim([.8,1.2]); % ylim([.8,1.2]);
xlim([0,t_max]); % xlim([.65,.76]);
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Solution updates (only useful to compare performance of different initial solutions of branch-and-bound)
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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

title('Solution Updates')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Iterations [10]')
legend('Torque reference','# Iterations opt','# Iterations ed','# Iterations sdp');
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Visited parent nodes
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
nc0 = reshape([zeros(1,length(node_count)); node_count(1,:); zeros(1,length(node_count))],1,3*length(node_count));
nc1 = reshape([zeros(1,length(node_count)); node_count(2,:); zeros(1,length(node_count))],1,3*length(node_count));
nc2 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
nc3 = reshape([zeros(1,length(node_count)); node_count(4,:); zeros(1,length(node_count))],1,3*length(node_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(7);
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold on;
% plot(tc,nc0/1000,'b');
plot(tc+ctrl0.T_s/6, nc1/1000,'r');
% plot(tc+ctrl0.T_s*2/6, nc0/1000,'b');
plot(tc+ctrl0.T_s*3/6, nc3/1000,'g');
hold off;

title('Nodes')
xlim([.01,t_max]);
xlabel('time [s]')
ylabel('torque [pu] / # nodes [10^3]')
% legend('Torque reference','opt', ctrl1.type, ctrl2.type, ctrl3.type, 'Location','northwest');
legend('Torque reference', ctrl1.type, ctrl3.type, 'Location', 'northwest');
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Time
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
nc = reshape([zeros(1,length(time_count)); time_count(1,:); zeros(1,length(time_count))],1,3*length(time_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc2 = reshape([zeros(1,length(time_count)); time_count(2,:); zeros(1,length(time_count))],1,3*length(time_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc3 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
tc3 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(8);
plot(t_ctrl_vec, ref(1,1:end-1)/100,'k');
hold on;
plot(tc,nc,'r');
plot(tc2+ctrl0.T_s/6,nc2,'b');
plot(tc3+ctrl0.T_s*2/6, nc3,'g');
hold off;

title('Time')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Time [s]')
legend('Torque reference','# Nodes no limit','# Nodes ed guess', '# Nodes ed guess + sdp');
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Accumulated cost
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
J_acc_0 = cost_vec(1,:);
J_acc_1 = cost_vec(ctrl0.n_costs + 1,:);
J_acc_2 = cost_vec(ctrl0.n_costs+ctrl1.n_costs+1,:);
J_acc_3 = min(cost_vec(ctrl0.n_costs+ctrl1.n_costs+ctrl2.n_costs+...
    [1:ctrl3.n_costs],:));
for k = 2:length(J_acc_0)
    J_acc_0(k) = J_acc_0(k) + J_acc_0(k-1);
    J_acc_1(k) = J_acc_1(k) + J_acc_1(k-1);
    J_acc_2(k) = J_acc_2(k) + J_acc_2(k-1);
    J_acc_3(k) = J_acc_3(k) + J_acc_3(k-1);
end

figure(9);
plot(t_ctrl_vec, ref_ctrl,'k');
hold on;
plot(t_ctrl_vec,J_acc_0/100,'b');
plot(t_ctrl_vec,J_acc_1/100,'r');
% plot(t_ctrl_vec,J_acc_2/100,'m');
plot(t_ctrl_vec,J_acc_3/100,'g');
hold off;

title('Accumulated Cost')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / Accumulated Cost [10]')
legend('Torque reference','opt', ctrl1.type, ctrl2.type, ctrl3.type, 'Location','northwest');
grid on;

% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Currents
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

K_inv = [1 0; -.5 sqrt(3)/2; -.5 -sqrt(3)/2];
i_s = sys.X_r/sys.D*x_vec_sim(1:2,:,:) - sys.X_m/sys.D*x_vec_sim(3:4,:,:);
figure(10);
plot(t_ctrl_vec, K_inv*squeeze(i_s(:,1,1:50:end-1)),'b');
hold on;
plot(t_ctrl_vec, K_inv*squeeze(i_s(:,2,1:50:end-1)),'r');
plot(t_ctrl_vec, K_inv*squeeze(i_s(:,4,1:50:end-1)),'g');
hold off;

grid on;


% *************************************************************************
%% Printing
% *************************************************************************
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
del_u3 = abs([sim.u_0 squeeze(u_vec(:,4,1:end-1))] - squeeze(u_vec(:,4,:)));
f_sw3 = 1/(12*t_max)*sum(del_u3, 'all');
fprintf(['Average switching frequency (','opt','):      %.2fHz \n'], f_sw0);
fprintf(['Average switching frequency (',ctrl1.type,'): %.2fHz \n'], f_sw1);
fprintf(['Average switching frequency (',ctrl2.type,'): %.2fHz \n'], f_sw2);
fprintf(['Average switching frequency (',ctrl3.type,'): %.2fHz \n\n'], f_sw3);

% Print torque rms error
T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));
fprintf(['Torque rms error (','opt','):      %.4fe-3 \n'], ...
    norm(T_sim(1,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf(['Torque rms error (',ctrl1.type,'): %.4fe-3 \n'], ...
    norm(T_sim(2,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf(['Torque rms error (',ctrl2.type,'): %.4fe-3 \n'], ...
    norm(T_sim(3,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
fprintf(['Torque rms error (',ctrl3.type,'): %.4fe-3 \n\n'], ...
    norm(T_sim(4,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);

% Print absolute flux rms error
Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));
fprintf(['Absolute flux rms error (','opt','):      %.4fe-3 \n'], ...
    norm(Psi_sim(1,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf(['Absolute flux rms error (',ctrl1.type,'): %.4fe-3 \n'], ...
    norm(Psi_sim(2,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf(['Absolute flux rms error (',ctrl2.type,'): %.4fe-3 \n'], ...
    norm(Psi_sim(3,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);
fprintf(['Absolute flux rms error (',ctrl3.type,'): %.4fe-3 \n\n'], ...
    norm(Psi_sim(4,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);

% Print traversed nodes
fprintf('Traversed nodes with opt guess in [# nodes] (mean, std, max): %.1f, %.1f, %.0f \n',...
    mean(node_count(2,2e4:end)), ...
    std(node_count(2,2e4:end)), ...
    max(node_count(2,2e4:end)));
fprintf('Traversed nodes with ed. guess in [# nodes] (mean, std, max): %.1f, %.1f, %.0f \n',...
    mean(node_count(1,2e4:end)), ...
    std(node_count(1,2e4:end)), ...
    max(node_count(1,2e4:end)));
fprintf('Traversed nodes with bad guess in [# nodes] (mean, std, max): %.1f, %.1f, %.0f \n',...
    mean(node_count(3,2e4:end)), ...
    std(node_count(3,2e4:end)), ...
    max(node_count(3,2e4:end)));

% % Current TDD
% i_s = sys.X_r/sys.D*x_vec_sim(1:2,:,:) - sys.X_m/sys.D*x_vec_sim(3:4,:,:);
% fprintf('Current TDD of controller %.0f: %.2f\n',1,compute_TDD(sim,sys,i_s,1));
% fprintf('Current TDD of controller %.0f: %.2f\n',1,compute_TDD(sim,sys,i_s,2));
% fprintf('Current TDD of controller %.0f: %.2f\n',1,compute_TDD(sim,sys,i_s,3));
% fprintf('Current TDD of controller %.0f: %.2f\n',1,compute_TDD(sim,sys,i_s,4));

% % Print accumulated cost
% fprintf('Average tracking cost: (ctrl0)       %.4fe-3 \n', ...
%     J_acc_0(end)/length(J_acc_0)*1e3);
% fprintf('Average tracking cost: (ctrl1)       %.4fe-3 \n', ...
%     J_acc_1(end)/length(J_acc_1)*1e3);
% fprintf('Average tracking cost: (ctrl2)       %.4fe-3 \n', ...
%     J_acc_2(end)/length(J_acc_2)*1e3);



