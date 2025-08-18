

T_rms = cell(n_sims,1);
Psi_rms = cell(n_sims,1);
T_rms_after_step = cell(n_sims,1);
Psi_rms_after_step = cell(n_sims,1);

for sim_cnt = 1:n_sims
    % Some parameters for easier plotting and printing
    % --- Time parameters -----------------------------------------------------
    t_ctrl_vec = double(0:n_controller_samples-1)*ctrl0.T_s;
    t_sim_vec = double(0:n_simulation_samples-1)*sim.T_sim;
    t_max = double(n_controller_samples-1)*ctrl0.T_s;
    % --- Simulation parameters -----------------------------------------------
    psi_s_sim = x_vec_sims{sim_cnt}(1:2,:,:);
    psi_r_sim = x_vec_sims{sim_cnt}(3:4,:,:);
    ref_sim = interp1([t_ctrl_vec double(n_controller_samples)*ctrl0.T_s], ...
        ref_sims{sim_cnt}(:,:)', [t_sim_vec double(n_simulation_samples)*sim.T_sim])';
    ref_sim = ref_sim(:,1:end-1);
    % --- Controller parameters -----------------------------------------------
    x_vec_ctrl = x_vec_sims{sim_cnt}(:,:,1:simulation_samples_per_controller_sample:end);
    psi_s_ctrl = x_vec_ctrl(1:2,:,:);
    psi_r_ctrl = x_vec_ctrl(3:4,:,:);
    ref_ctrl = ref_sims{sim_cnt}(1,1:end-1);
    %
    steps = cellfun(@(c) c{1}, steps_pre{sim_cnt}, 'UniformOutput', false);
    
    % Compute the torque error
    T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));
    T_rms{sim_cnt} = round( ...
        [norm(T_sim(2,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3, ...
         norm(T_sim(4,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3], ...
        2 ...
    );
    
    % Compute the absolute stator flux error
    Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));
    Psi_rms{sim_cnt} = round([norm(Psi_sim(2,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3, ...
                        norm(Psi_sim(4,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3],2);

    % Compute the torque error after each step
    T_rms_after_step{sim_cnt} = zeros(n_steps,2);
    Psi_rms_after_step{sim_cnt} = zeros(n_steps,2);
    for step_cnt = 1:n_steps
        step_start_idx = steps{step_cnt+1}(2)*simulation_samples_per_controller_sample;
        step_end_idx = step_start_idx + round((measurement_length+.01)/sim.T_sim);
        step_idx_interval = step_start_idx:step_end_idx;
        T_rms_after_step{sim_cnt}(step_cnt+1,:) = ...
            [norm(T_sim(2,step_idx_interval)-ref_sim(1,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3;
             norm(T_sim(4,step_idx_interval)-ref_sim(1,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3];
        Psi_rms_after_step{sim_cnt}(step_cnt+1,:) = ...
            [norm(Psi_sim(2,step_idx_interval)-ref_sim(2,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3;
             norm(Psi_sim(4,step_idx_interval)-ref_sim(2,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3];
    end
end

% --- RMS errors over full simulation horizon ----
% Printing
fprintf( "\n--------------------------------\n")
fprintf(   " RMS errors over full simulation\n")
fprintf(   "--------------------------------\n")
fprintf(  "\n Simulation count   |" + " Torque rms error  |" + " Absolute stator flux rms error\n")
fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")
for sim_cnt = 1:n_sims
    fprintf(" Sim " + num2str(sim_cnt) + ":" + " (b&b only)  | " + num2str(T_rms{sim_cnt}(1)) + "*1e-3       | " + num2str(Psi_rms{sim_cnt}(1)) + "*1e-3 \n");
    fprintf("        (SDP + b&b) | " + num2str(T_rms{sim_cnt}(2)) + "*1e-3       | " + num2str(Psi_rms{sim_cnt}(2)) + "*1e-3 \n");
    fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")
end

% --- RMS error mean and std for all steps over all simulations ----
% Computation
T_vals = zeros(n_sims*n_steps,2);
Psi_vals = zeros(n_sims*n_steps,2);
idx = 1;
for sim_cnt = 1:n_sims
    for step_cnt = 1:n_steps
        T_vals(idx,:) = T_rms_after_step{sim_cnt}(step_cnt,:);
        Psi_vals(idx,:) = Psi_rms_after_step{sim_cnt}(step_cnt,:);
        idx = idx + 1;
    end
end
T_mean = round(mean(T_vals),2);
T_std = round(std(T_vals),2);
Psi_mean = round(mean(Psi_vals),2);
Psi_std = round(std(Psi_vals),2);

% Printing
fprintf( "\n----------------------\n")
fprintf(   " RMS error means & std\n")
fprintf(   "----------------------\n")
fprintf(  "\n Simulation count   |" + " Torque rms error  |" + " Absolute stator flux rms error\n")
fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")

fprintf(" Means: (b&b only)  | " + num2str(T_mean(1)) + "*1e-3       | " + num2str(Psi_mean(1)) + "*1e-3 \n");
fprintf("        (SDP + b&b) | " + num2str(T_mean(2)) + "*1e-3       | " + num2str(Psi_mean(2)) + "*1e-3 \n");
fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")

fprintf(" Stds:  (b&b only)  | " + num2str(T_std(1)) + "*1e-3       | " + num2str(Psi_std(1)) + "*1e-3 \n");
fprintf("        (SDP + b&b) | " + num2str(T_std(2)) + "*1e-3       | " + num2str(Psi_std(2)) + "*1e-3 \n");
fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")

% fprintf(['Torque rms error (','opt','):      %.4fe-3 \n'], ...
%     norm(T_sim(1,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3);
% fprintf(['Absolute flux rms error (','opt','):      %.4fe-3 \n'], ...
%     norm(Psi_sim(1,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3);

