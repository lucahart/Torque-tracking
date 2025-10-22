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
ref_sim = interp1([t_ctrl_vec double(n_controller_samples)*ctrl0.T_s], ref(:,:)', [t_sim_vec double(n_simulation_samples)*sim.T_sim])';
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

figure(2)
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

end

% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot torque
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Plot stator flux
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Solumtion updates (only useful to compare performance of different initial solutions of branch-and-bound)
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Nodes
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
nc = reshape([zeros(1,length(node_count)); node_count(1,:); zeros(1,length(node_count))],1,3*length(node_count));
tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc1 = reshape([zeros(1,length(node_count)); node_count(2,:); zeros(1,length(node_count))],1,3*length(node_count));
tc1 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
nc2 = reshape([zeros(1,length(node_count)); node_count(3,:); zeros(1,length(node_count))],1,3*length(node_count));
tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));

figure(7);
plot(t_ctrl_vec, ref(1,1:end-1),'k');
hold on;
plot(tc,nc2/1e3,'g');
plot(tc1+ctrl0.T_s/6,nc/1e3,'b');
plot(tc2+ctrl0.T_s/3,nc1/1e3,'r');
hold off;

title('# Parent nodes traversed')
xlim([0,t_max]);
xlabel('time [s]')
ylabel('Torque [pu] / # Nodes [10^3]')
legend('Torque reference', 'Bad guess', 'Ed guess', 'Opt guess');
grid on;


% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% Time
% ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
% nc = reshape([zeros(1,length(time_count)); time_count(1,:); zeros(1,length(time_count))],1,3*length(time_count));
% tc = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
% J_acc_2 = reshape([zeros(1,length(time_count)); time_count(2,:); zeros(1,length(time_count))],1,3*length(time_count));
% tc2 = reshape([t_ctrl_vec;t_ctrl_vec;t_ctrl_vec],1,3*length(t_ctrl_vec));
% 
% figure(8);
% plot(t_ctrl_vec, ref(1,1:end-1),'k');
% hold on;
% plot(tc,nc,'r');
% plot(tc2+ctrl0.T_s/6,J_acc_2,'b');
% hold off;
% 
% title('Time')
% xlim([0,t_max]);
% xlabel('time [s]')
% ylabel('Torque [pu] / # Time [s]')
% legend('Torque reference', 'Ed guess', 'Opt guess');
% grid on;


% *************************************************************************
%% Printing
% *************************************************************************
fprintf('\n--------------------------------------------------------\n');

% Print elapsed simulation time
fprintf('Elapsed simulation time:     %.2fs \n\n', t_sim);

% Print average switching frequency
del_u0 = abs([sim.u_0 squeeze(u_vec(:,1,1:end-1))] - squeeze(u_vec(:,1,:)));
f_sw0 = 1/(12*t_max)*sum(del_u0, 'all');
fprintf('Average switching frequency:            %.2fHz \n', f_sw0);
% Print torque rms error
fprintf('Torque rms error:                       %.4fe-3 \n', ...
    norm(T_sim(1,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);

% Print absolute flux rms error
fprintf('Absolute flux rms error:                %.4fe-3 \n\n', ...
    norm(Psi_sim(1,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);

% Parent nodes visited
fprintf('---------------------------------------\n')
fprintf('       Bad guess   Ed guess   Opt guess\n')
fprintf('---------------------------------------\n')
fprintf('mean:  %.2f       %.2f       %.2f\n', ...
    mean(node_count(3,:)), mean(node_count(1,:)), mean(node_count(2,:)))
fprintf('std:   %.2f       %.2f      %.2f\n', ...
    std(node_count(3,:)), std(node_count(1,:)), std(node_count(2,:)))
fprintf('max:   %.0f        %.0f        %.0f\n', ...
    max(node_count(3,:)), max(node_count(1,:)), max(node_count(2,:)))
fprintf('---------------------------------------\n')


