function [a, s] = generate_polynomial(n)
% GENERATE_POLYNOMIAL creates a random polynomial of degree 2 and 
% dimension n.
% Input
%   n: Polynomial dimension.
% Output
%   a: Coefficients of the polynomial.
%   s: String representation of the polynomial p(x).

% Set hyperparameters
p_zero = .1; % pr. of setting coefficients to 0 (real pr. is higher)
l = -1; % lower bound of coefficients
u = 1; % upper bound of coefficients
prec = 2; % decimal precision

%% Generate polynomial coefficients
m = ((n+1)^2 + n + 1)/2; % length of a
a = round(rand(m,1)*(u-l)+l, prec); % create random a
a(rand(m,1) < p_zero) = 0; % set coefficients to 0


%% String s to print polynomial
s = '';
v = "";
k = 1;
for i = 1:n
    v = [v, append('x',num2str(i))];
end
% v
for i = 1:n+1
    for j = i:n+1
        % remove all 0 entries
        if a(k) == 0
            k = k + 1;
            continue;
        end
        % no complicated stuff for constant offset
        if k == 1            
            s = num2str(a(k));
        else            
            % set the sign
            if a(k) < 0
                sign = ' - ';
            else
                sign = ' + ';
            end
            % set squared terms
            if i == j
                var = append(v(i), '^2');
            else
                var = append(v(i), v(j));
            end
            s = append(s, sign, num2str(abs(a(k))), var);
        end
        k = k + 1;
    end
end

end