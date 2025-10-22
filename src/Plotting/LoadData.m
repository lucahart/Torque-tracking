% Read the data (automatically parses column headers)
data = readtable('weighted_average_error.txt');

% Compute summary statistics for all columns
mean_vals = varfun(@mean, data);
quartiles = varfun(@(x) quantile(x, [0.25 0.5 0.75]), data);
min_vals = varfun(@min, data);
max_vals = varfun(@max, data);

% Display the results
disp(mean_vals)
disp(quartiles)
disp(min_vals)
disp(max_vals)
