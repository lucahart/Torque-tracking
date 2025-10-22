% Export data to txt files
t_sim_vec = double(0:n_simulation_samples-1)*sim.T_sim;

% Electric torque
T_sim = 1/sys.pf*sys.X_m/sys.D*(squeeze(psi_r_sim(1,:,:)).*squeeze(psi_s_sim(2,:,:)) - squeeze(psi_r_sim(2,:,:)).*squeeze(psi_s_sim(1,:,:)));
varNames = {'T_eref','T_el','T_e2','T_e3','t'};
tab = table(ref(1,1:end-1)', T_sim(1,1:50:end)', T_sim(2,1:50:end)', T_sim(4,1:50:end)', t_sim_vec(1:50:end)', 'VariableNames', varNames);
writetable(tab(1:10:end,:),'torque.txt','Delimiter',' ');

% Stator flux magnitude
Psi_sim = squeeze(vecnorm(psi_s_sim,2,1));

varNames = {'Psi_sref','Psi_s1','Psi_s2','Psi_s3','t'};
tab=table(ref(2,1:end-1)', Psi_sim(1,1:50:end)', Psi_sim(2,1:50:end)', Psi_sim(4,1:50:end)', t_sim_vec(1:50:end)','VariableNames',varNames);
writetable(tab(1:10:end,:),'stator_flux.txt','Delimiter',' ');


