function [x_values] = bounded_random_walk(x_init,x_min,x_max,nSteps)
% boundedRandomWalk  Move a point in [-1,1] by random steps (default 10)
%
%   [x,step] = boundedRandomWalk(x_init,a,b)          % 10 steps
%   [x,step] = boundedRandomWalk(x_init,a,b,nSteps)   % nSteps steps
%
%   x_init   – starting location (will be clipped to [-1,1])
%   a,b      – positive scalars with a ≤ b (step interval parameters)
%   nSteps   – (optional) number of steps, default 10
%
%   x        – row vector of positions, length nSteps+1
%   step     – row vector of realised steps, length nSteps

% ---------- input checks -------------------------------------------------
if nargin < 4, nSteps = 10; end
validateattributes(x_init, {'numeric'},{'scalar'});
validateattributes(x_min,  {'numeric'},{'scalar','positive'});
validateattributes(x_max,  {'numeric'},{'scalar','positive','>=',x_min});
validateattributes(nSteps, {'numeric'},{'scalar','integer','positive'});


% Iterate to create an nStep simulation
x_values = nan(nSteps+1,1);
x_values(1) = x_init;
x_temp = x_init;
for step = 1:nSteps

    % ---------- compute feasible step intervals --------------------------
    % Lowest and highest feasible DIFERENCE when stepping down
    if (x_temp - x_min >= -1) 
        del_down_high = -x_min; 
        if (x_temp - x_max >= -1) 
            del_down_low = -x_max; 
        else 
            del_down_low = -1-x_temp; 
        end
    else 
        del_down_high = 0; 
        del_down_low = 0;
    end
    
    % Lowest and highest feasible value when stepping up (no difference here)
    up_low = min(x_temp + x_min, 1);
    up_high = min(x_temp + x_max, 1);
    
    % Length of feasible intervals if stepping up/down
    del_down = abs(del_down_high - del_down_low);
    del_up = abs(up_high - up_low);
    
    % --------- update perform random walk step ---------------------------
    v = rand;
    if v - del_down <= 0
        x_temp = x_temp + v + del_down_low;
    else
        x_temp = v + min(up_low, 0);
    end

    % Store results
    x_values(step+1) = x_temp;
end

