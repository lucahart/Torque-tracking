% Semidefinite Programming Relaxations and Algebraic Optimization in
% Control, Pablo A. Parrilo and Sanjay Lall, ECC

%% SDP Relaxations for Quadratic Programming
yalmip('clear');

% Introduction of optimization problem
Q = [0 1 2; 1 0 3; 2 3 0];

%% Primal
x = sdpvar(3,1);
X = x*x';
F = [
    X >= 0;
%     X(1) == 1;
%     X(5) == 1;
%     X(9) == 1;
];
optimize(F); %,[],sdpsettings('debug',1)
value(X)

%% Dual
Lambda = sdpvar(3);
D = [
    Q >= Lambda,
    Lambda(2) == 0,
    Lambda(3) == 0,
    Lambda(6) == 0,
];
optimize(D, -trace(Lambda));
value(Lambda)


%% Create polynomials
% Set hyperparameters
n = 4; % dimension of polynomial
p_zero = .3; % pr. of setting coefficients to 0 (real pr. is higher)
l = -10; % lower bound of coefficients
u = 10; % upper bound of coefficients
prec = 0; % decimal precision

% Create polynomial
m = ((n+1)^2 + n + 1)/2; % length of a
a = round(rand(m,1)*(u-l)+l, prec); % create random a
a(rand(m,1) < p_zero) = 0; % set coefficients to 0

% Print polynomial
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
display(s)


%% First attempts for Automation
yalmip('clear');

% Introduction of optimization problem
% a = [0 0 0 0 0 1 2 0 3 0];
% n = 3;
Q = decompose(a,n);
% Primal
x = sdpvar(n+1,1);
X = x*x';
F = [X >= 0];
% F = [F, X(1) == 1];
for i = 1:n+1
    F = [F, X(i,i) == 1];
end
optimize(F, trace(Q*X));
value(X)
value(x)



