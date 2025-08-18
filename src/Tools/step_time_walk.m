function step_times = step_time_walk(step_time_min, n_steps, sim_length)

    % Draw random samples of the step time differences
    step_times = step_time_min + rand(n_steps+1,1)*(1-step_time_min);
    
    % Add up step time differences to step times
    for step = 1:n_steps
        step_times(step+1) = step_times(step+1) + step_times(step);
    end
    
    % Normalize by the simulation length
    step_times(:) = step_times(:) * sim_length/step_times(end);
end
