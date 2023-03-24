function ref = generate_reference(steps, ramps, n_controller_samples, factor)
    % GENERATE_REFERENCE takes specifications like steps and ramps for the
    % reference into account and computes a full reference vector.
    % Input:
    %   steps: Cell-object that specifies the steps within the reference.
    %     Format: steps{i} = [r, k, v], where
    %       r \in {1,2}: specifies the reference (torque or abs. st. flux)
    %       k \in {1,...,n_controller_samples}: time when step is applied
    %       v \in [-1,1]: value that the reference takes at the step
    %   ramps: Cell-object that specifies the ramps within the reference.
    %     Format: ramps{i} = [r, k1, v1, k2, v2], where
    %       r \in {1,2}: specifies the reference (torque or abs. st. flux)
    %       k1 \in {1,...,n_controller_samples}: time when ramp starts
    %       v1 \in [-1,1]: value that the reference starts at
    %       k2 \in {1,...,n_controller_samples}: time when ramp ends
    %       v2 \in [-1,1]: value that the reference end with
    %   n_controller_samples: number of samples that the controller is
    %     executed in the simulation.
    %   factor (optional): Factor for simplifying the specificatin of
    %     times. E.g. choose factor = 1/ctrl.T_s to change time
    %     specification to seconds, or factor = 1/f_1 to change the time
    %     specification into fundamentals.
    %     Default is factor = 1, where time represents controller samples.

    % Use a factor for specifying the step time. E.g. choose factor = 1/T_s
    % to change the time step specification to one in seconds
    if nargin < 4
        factor = 1;
    end

    % Generate reference array
    ref = ones(2,n_controller_samples);

    % Set reference steps as specified by the steps cell-object:
    % steps{i} = [r, k, v], x \in {1,2}: which reference value,
    % r \in {1,...,n_controller_samples}: time step of step,
    % k \in [-1,1]: value that the reference takes at the step
    for i = 1:length(steps)
        step = steps{i};
        % Avoid extending the ref. over the simulation length
        if step(2)*factor > n_controller_samples
            break;
        end
        
        ref(step(1), step(2)*factor:end) = step(3);
    end
    
    % Set reference ramps as specified by the ramps cell-object:
    % ramps{i} = [r, k1, v1, k2, v2], r \in {1,2}: which reference value,
    % k1 \in {1,...,n_controller_samples}: time step where ramp starts
    % v1 \in [-1,1]: value that the ramp takes in the beginning
    % k2 \in {y1,...,n_controller_samples}: time steps where ramp ends
    % v2 \in [-1,1]: value that the ramp takes in the end
    for i = 1:length(ramps)
        ramp = ramps{i};
        start = ramp(2)*factor;
        % Avoid extending the ref. over the simulation length
        if ramp(4)*factor > n_controller_samples
            break;
        end
        % Avoid rounding errors
        if start <= 0
            start = 1;
        end
        
        ref(ramp(1), start:ramp(4)*factor) = ...
            linspace(ramp(3),ramp(5),ramp(4)*factor-start+1);
    end
    
    ref = [ref(:,1) ref];
    
end