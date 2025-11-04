%% Setup Script
% Check if SCS is available
if (exist('scs','file') ~= 2)
    error("SCS could not be found.");
end

% Add paths for used functions
addpath('.')
addpath(genpath('./src'))
addpath(genpath('./YALMIP-R20210331'))

% Overwrite outdated functions in YALMIP for SCS compatibility
addpath('./src/Tools/','-begin')
