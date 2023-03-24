function [xvec_ss,uvec_ss] = results(xvec,uvec,sph_count,Tsim,number_of_samples_per_fundamental,controller_samples_per_fundamental,ramp2start,periods_to_wait_for_ss,K_inv,sys)
% RESULTS is a deprecated function that is not in use anymore. It can
% return the current TDD of a specified current and vector.

warning('The results-function is deprecated.');


xvec_ss = xvec;
uvec_ss = uvec;

% number_of_samples_per_fundamental = controller_samples_per_fundamental; %TODO: not true, either replace all n fund. or use it :)d

xvec_ss(:,1:number_of_samples_per_fundamental*periods_to_wait_for_ss+1) = [];
xvec_ss(:,length(xvec_ss)-ramp2start*number_of_samples_per_fundamental:end) = [];
uvec_ss(:,1:floor(controller_samples_per_fundamental*periods_to_wait_for_ss)) = [];
uvec_ss(:,length(uvec_ss)-ramp2start*controller_samples_per_fundamental:end) = [];

% fsw = 0;
% for i=1:1:length(uvec_ss)-1
%     fsw = fsw + norm(uvec_ss(:,i+1) - uvec_ss(:,i),1);
% end
% fsw = fsw/(12*length(uvec_ss)*Tsim);

% % 
i_g = sys.K_inv*((sys.X_s - sys.X_m/sys.X_r*sys.X_m)\(xvec_ss(1:2,:) - sys.X_m/sys.X_r*xvec_ss(3:4,:)));

dataLength = length(xvec_ss);
rawFFT = (fft(i_g'))';
doubleSided = abs(rawFFT/dataLength);
singleSided = doubleSided(:,1:floor(dataLength/2)+1);
singleSided(:,2:end-1) = 2*singleSided(:,2:end-1);
singleSidedToPlot = singleSided;
single_sided_i_ga = singleSided(1,:);
single_sided_i_gb = singleSided(2,:);
single_sided_i_gc = singleSided(3,:);
peak_fund_i_ga = 1;%max(single_sided_i_ga);
peak_fund_i_gb = 1;%max(single_sided_i_gb);
peak_fund_i_gc = 1;%max(single_sided_i_gc);
single_sided_i_g_a_to_plot = single_sided_i_ga;
single_sided_i_g_b_to_plot = single_sided_i_gb;
single_sided_i_g_c_to_plot = single_sided_i_gc;
% single_sided_i_ga(single_sided_i_ga==max(single_sided_i_ga)) = []; % remove fundamental from harmonics
% single_sided_i_gb(single_sided_i_gb==max(single_sided_i_gb)) = [];
% single_sided_i_gc(single_sided_i_gc==max(single_sided_i_gc)) = [];
[~, peak_idx] = max(single_sided_i_ga);
window_width = 1; % [Hz]
peak_interval = floor(window_width/2*dataLength*Tsim);
% peak_interval = 3;

single_sided_i_ga((-peak_interval:peak_interval)+peak_idx) = [];
single_sided_i_gb((-peak_interval:peak_interval)+peak_idx) = [];
single_sided_i_gc((-peak_interval:peak_interval)+peak_idx) = [];

% single_sided_i_ga(1:2*peak_idx-10) = [];
% single_sided_i_gb(1:2*peak_idx-10) = [];
% single_sided_i_gc(1:2*peak_idx-10) = [];

single_sided_i_ga_harmonics = single_sided_i_ga(2*peak_idx:peak_idx:end);
single_sided_i_gb_harmonics = single_sided_i_gb(2*peak_idx:peak_idx:end);
single_sided_i_gc_harmonics = single_sided_i_gc(2*peak_idx:peak_idx:end);

current_THD_i_g_a = norm(single_sided_i_ga,2)/peak_fund_i_ga*100;
current_THD_i_g_b = norm(single_sided_i_ga,2)/peak_fund_i_gb*100;
current_THD_i_g_c = norm(single_sided_i_ga,2)/peak_fund_i_gc*100;

current_THD = norm((current_THD_i_g_a+current_THD_i_g_b+current_THD_i_g_c)/3);
fft_freqs = (0:dataLength/2)/dataLength/Tsim;

[~,max_index] = min(abs(fft_freqs-2500));

figure(10);
plot(fft_freqs(1:max_index),single_sided_i_g_a_to_plot(1:max_index));
axis([0 2500 0 1])

fprintf('Current TDD:                 %0.4f%%  \n',current_THD)
% fprintf('Average switching freq: %0.5f Hertz \n',fsw)
% fprintf('Mean sphere decoding nodes: %0.5f \n',mean(sph_count))
end
