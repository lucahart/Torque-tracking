function ref = generateReference(steps, ramps, n_controller_samples, factor)


    % Use a factor for specifying the step time. E.g. choose factor = 1/T_s
    % to change the time step specification to one in seconds
    if nargin < 4
        factor = 1;
    end

    % Generate reference array
    ref = ones(2,n_controller_samples);

    % Set reference steps as specified by the steps cell-object:
    % steps{i} = [x, y, v], x \in {1,2}: which reference value,
    % y \in {1,...,n_controller_samples}: time step of step,
    % v \in [-1,1]: value that the reference takes at the step
    for i = 1:length(steps)
        step = steps{i};
        ref(step(1), step(2)*factor:end) = step(3);
    end
    
    % Set reference ramps as specified by the ramps cell-object:
    % ramps{i} = [x, y1, v1, y2, v2], x \in {1,2}: which reference value,
    % y1 \in {1,...,n_controller_samples}: time step where ramp starts
    % v1 \in [-1,1]: value that the ramp takes in the beginning
    % y2 \in {y1,...,n_controller_samples}: time steps where ramp ends
    % v2 \in [-1,1]: value that the ramp takes in the end
    for i = 1:length(ramps)
        ramp = ramps{i};
        start = ramp(2)*factor;
        if start <= 0
            start = 1;
        end
        ref(ramp(1), start:ramp(4)*factor) = ...
            linspace(ramp(3),ramp(5),ramp(4)*factor-start+1);
    end
    
    ref = [ref ref(:,end)];
    
end