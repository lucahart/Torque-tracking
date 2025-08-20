
%% Precompute performance values
T_rms = cell(n_sims,n_node_limits);
Psi_rms = cell(n_sims,n_node_limits);
T_rms_after_step = cell(n_sims,n_node_limits);
Psi_rms_after_step = cell(n_sims,n_node_limits);

for node_lim_cnt = 1:n_node_limits
    for sim_cnt = 1:n_sims
        % Some parameters for easier plotting and printing
        % --- Time parameters -----------------------------------------------------
        t_ctrl_vec = double(0:n_controller_samples-1)*ctrl0.T_s;
        t_sim_vec = double(0:n_simulation_samples-1)*sim.T_sim;
        t_max = double(n_controller_samples-1)*ctrl0.T_s;
        % --- Simulation parameters -----------------------------------------------
        psi_s_sim = x_vec_sims{sim_cnt,node_lim_cnt}(1:2,:,:);
        psi_r_sim = x_vec_sims{sim_cnt,node_lim_cnt}(3:4,:,:);
        ref_sim = interp1([t_ctrl_vec double(n_controller_samples)*ctrl0.T_s], ...
            ref_sims{sim_cnt,node_lim_cnt}(:,:)', [t_sim_vec double(n_simulation_samples)*sim.T_sim])';
        ref_sim = ref_sim(:,1:end-1);
        % --- Controller parameters -----------------------------------------------
        x_vec_ctrl = x_vec_sims{sim_cnt,node_lim_cnt}(:,:,1:simulation_samples_per_controller_sample:end);
        psi_s_ctrl = x_vec_ctrl(1:2,:,:);
        psi_r_ctrl = x_vec_ctrl(3:4,:,:);
        ref_ctrl = ref_sims{sim_cnt,node_lim_cnt}(1,1:end-1);
        %
        steps = cellfun(@(c) c{1}, steps_pre{sim_cnt}, 'UniformOutput', false);
        
        % Compute the torque error
        T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));
        T_rms{sim_cnt,node_lim_cnt} = round( ...
            [norm(T_sim(2,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3, ...
             norm(T_sim(4,:)-ref_sim(1,:),2)/sqrt(length(T_sim))*1e3], ...
            2 ...
        );
        
        % Compute the absolute stator flux error
        Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));
        Psi_rms{sim_cnt,node_lim_cnt} = round([norm(Psi_sim(2,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3, ...
                            norm(Psi_sim(4,:)-ref_sim(2,:),2)/sqrt(length(Psi_sim))*1e3],2);
    
        % Compute the torque error after each step
        T_rms_after_step{sim_cnt,node_lim_cnt} = zeros(n_steps,2);
        Psi_rms_after_step{sim_cnt,node_lim_cnt} = zeros(n_steps,2);
        for step_cnt = 1:n_steps
            step_start_idx = steps{step_cnt+1}(2)*simulation_samples_per_controller_sample;
            step_end_idx = step_start_idx + round((measurement_length+.01)/sim.T_sim);
            step_idx_interval = step_start_idx:step_end_idx;
            T_rms_after_step{sim_cnt,node_lim_cnt}(step_cnt+1,:) = ...
                [norm(T_sim(2,step_idx_interval)-ref_sim(1,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3;
                 norm(T_sim(4,step_idx_interval)-ref_sim(1,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3];
            Psi_rms_after_step{sim_cnt,node_lim_cnt}(step_cnt+1,:) = ...
                [norm(Psi_sim(2,step_idx_interval)-ref_sim(2,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3;
                 norm(Psi_sim(4,step_idx_interval)-ref_sim(2,step_idx_interval),2)/sqrt(length(step_idx_interval))*1e3];
        end
    end
end


%% Print Results
figure(1); hold off;
figure(2); hold off;
n_all_steps = n_sims*n_steps;
for node_lim_cnt = 1:n_node_limits
    fprintf("\n--------------------------------------------------\n")
    fprintf("--------------------------------------------------\n")
    fprintf(" b&b+SDP node limit = %4d, b&b node limit = %4d\n", [250,branch_and_bound_node_limits(node_lim_cnt)])
    fprintf("--------------------------------------------------\n")
    fprintf("--------------------------------------------------\n")
    % --- RMS errors over full simulation horizon ----
    % Printing
    fprintf( "\n--------------------------------\n")
    fprintf(   " RMS errors over full simulation\n")
    fprintf(   "--------------------------------\n")
    fprintf(  "\n Simulation count   |" + " Torque rms error  |" + " Absolute stator flux rms error\n")
    fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")
    for sim_cnt = 1:n_sims
        fprintf(" Sim " + num2str(sim_cnt) + ":" + " (b&b only)  | " + num2str(T_rms{sim_cnt,node_lim_cnt}(1)) + "*1e-3       | " + num2str(Psi_rms{sim_cnt,node_lim_cnt}(1)) + "*1e-3 \n");
        fprintf("        (SDP + b&b) | " + num2str(T_rms{sim_cnt,node_lim_cnt}(2)) + "*1e-3       | " + num2str(Psi_rms{sim_cnt,node_lim_cnt}(2)) + "*1e-3 \n");
        fprintf(    "---------------------" + "--------------------" + "-------------------------------\n")
    end
    
    % --- RMS error mean and std for all steps over all simulations ----
    % Computation
    T_vals = zeros(n_all_steps,2);
    Psi_vals = zeros(n_all_steps,2);
    idx = 1;
    for sim_cnt = 1:n_sims
        for step_cnt = 1:n_steps
            T_vals(idx,:) = T_rms_after_step{sim_cnt,node_lim_cnt}(step_cnt+1,:);
            Psi_vals(idx,:) = Psi_rms_after_step{sim_cnt,node_lim_cnt}(step_cnt+1,:);
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

    % Box Plots for Torque
    figure(1);
    if node_lim_cnt == 1
        % Plot b&b Inf and SDP+b&b 250 in the first node limits
        boxchart(ones(n_all_steps,1), T_vals(:,1)*1e-3);
        hold on;
        boxchart(ones(n_all_steps,1)*2, T_vals(:,2)*1e-3);
    else
        boxchart(ones(n_all_steps,1)*(node_lim_cnt+1), T_vals(:,1)*1e-3);
    end

    % Box Plots for Absolute Stator Flux
    figure(2);
    if node_lim_cnt == 1
        % Plot b&b Inf and SDP+b&b 250 in the first node limits
        boxchart(ones(n_all_steps,1), Psi_vals(:,1)*1e-3);
        hold on;
        boxchart(ones(n_all_steps,1)*2, Psi_vals(:,2)*1e-3);
    else
        boxchart(ones(n_all_steps,1)*(node_lim_cnt+1), Psi_vals(:,1)*1e-3);
    end
end

% Create labels
labels = cell(n_node_limits+1,1);
labels{1} = 'b&b Inf'; 
labels{2} = 'SDP + b&b 250';
labels(3:n_node_limits+1) = arrayfun( ...
    @(k) sprintf('b&b %d', ...
    branch_and_bound_node_limits(k)), 2:n_node_limits, ...
    'UniformOutput', false ).';

% Plot labels for Torque Box Plot
figure(1);
xlim([0.5,n_node_limits+1.5]);
xticks(1:n_node_limits+1); xticklabels(labels)
xlabel('Solver Type & Node Limit')
ylabel(sprintf('RMS Tracking Error %2dms after step', ...
    measurement_length*1e3))
title('Torque T_e')
grid on;

% Plot labels for Absolute Stator Flux Box Plot
figure(2);
xlim([0.5,n_node_limits+1.5]);
xticks(1:n_node_limits+1); xticklabels(labels)
xlabel('Solver Type & Node Limit')
ylabel(sprintf('RMS Tracking Error %2dms after step', ...
    measurement_length*1e3))
title('Absolute Stator Flux \Psi')
grid on;


