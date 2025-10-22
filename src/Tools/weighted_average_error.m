%% weighted_average_error.m
% Combine two datasets column-wise with weights:
%   y = wa * (torque column) + wb * (flux column)
% Columns are matched by the part AFTER the first underscore.
% Example: T_250 + Psi_250 -> Y_250

% ----- USER SETTINGS -----
wa      = 0.052;                         % weight for torque_sweep.txt
wb      = 1 - wa;                         % weight for stator_flux_sweep.txt
fileA   = 'torque_sweep.txt';  % torque dataset (e.g., T_inf, T_250, ...)
fileB   = 'stator_flux_sweep.txt'; % flux dataset (e.g., Psi_inf, Psi_250, ...)
outFile = 'weighted_average_error.txt';  % output file path
% -------------------------

% --- Robust import for space-separated text with headers ---
optsA = detectImportOptions(fileA, 'Delimiter',' ', 'MultipleDelimsAsOne',true);
optsB = detectImportOptions(fileB, 'Delimiter',' ', 'MultipleDelimsAsOne',true);

% Ensure we read everything as numeric where possible
optsA = setvartype(optsA, 'double');
optsB = setvartype(optsB, 'double');

TA = readtable(fileA, optsA);
TB = readtable(fileB, optsB);

% --- Basic sanity checks ---
if height(TA) ~= height(TB)
    error('Row count mismatch: %d (A) vs %d (B). Make sure samples align.', height(TA), height(TB));
end

% Extract column "keys" by stripping the prefix up to the first underscore.
% e.g., T_250 -> 250, Psi_250 -> 250; T_inf -> inf, Psi_inf -> inf
keysA = regexprep(string(TA.Properties.VariableNames), '^[^_]*_', '');
keysB = regexprep(string(TB.Properties.VariableNames), '^[^_]*_', '');

% Build a map for quick lookup of columns in B by key
mapB = containers.Map(keysB, 1:numel(keysB));

% Find shared suffixes (order follows A to keep ticks/labels predictable)
commonKeys = intersect(keysA, keysB, 'stable');

if isempty(commonKeys)
    error('No matching columns. Check that column suffixes match across files (e.g., "_250", "_inf").');
end

% --- Compute weighted sums for all matching columns ---
Y = table();
for i = 1:numel(commonKeys)
    k    = commonKeys(i);
    idxA = find(keysA == k, 1, 'first');
    if ~isKey(mapB, k)
        warning('Key "%s" found in A but not in B. Skipping.', k);
        continue
    end
    idxB = mapB(k);

    colA = TA{:, idxA};   % torque column
    colB = TB{:, idxB};   % flux column

    % Weighted sum
    Y.(sprintf('e_%s', k)) = wa * colA + wb * colB;
end

% Optional: warn about unmatched columns
onlyA = setdiff(keysA, keysB);
onlyB = setdiff(keysB, keysA);
if ~isempty(onlyA)
    warning('Columns only in A (ignored): %s', strjoin(cellstr(onlyA), ', '));
end
if ~isempty(onlyB)
    warning('Columns only in B (ignored): %s', strjoin(cellstr(onlyB), ', '));
end

% --- Save as space-separated text with header ---
writetable(Y, outFile, 'Delimiter',' ', 'WriteVariableNames', true);

fprintf('Wrote weighted dataset (%d rows, %d columns) to: %s\n', height(Y), width(Y), outFile);
fprintf('Weights used: wa = %.6g, wb = %.6g\n', wa, wb);
