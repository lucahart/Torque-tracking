%% Setup
yalmip('clear');
opt = optimoptions('fmincon','Display','off');
eps = 1e-2;
sdpopt = sdpsettings('solver', 'scs', 'verbose', 0, 'dualize', 0, 'scs.eps_abs', eps, 'scs.eps_rel', eps);
prec = 1e-3;

%% Create random polynomial
n = 3*8;
[a, s] = generatePolynomial(n);

% Print polynomial
if n <= 4
    display(s)
end

Q = decompose(a, n); % Uncomment for good example
lb = -1;
ub = 1;

%% Create Q-Matrix

% Functional form of polynomial
p = @(x) x'*Q*x;

% Solve for minimum within constraints

x_hat = 0;
y_hat = inf;
x_opt = ones(4,0);
y_opt = ones(4,0);

t1 = tic;
for i = 1:100*n
    x0 = [1; rand(n,1)*(ub-lb) + lb];
    Aeq = [1, zeros(1,n)];
    beq = 1;
    [x, y] = fmincon(p,x0,[],[],Aeq,beq, ...
        lb*ones(n+1,1),ub*ones(n+1,1),[],opt);
    
%     if y < y_hat
%         x_hat = x;
%         y_hat = y;
%     end
    
    if i == 1
        x_opt = [x_opt, x];
        y_opt = [y_opt, y];
    end
    
    if ~any(all(x_opt - x < prec,1))
        x_opt = [x_opt, x];
        y_opt = [y_opt, y];
    end
        
end
t1 = toc(t1);

[y_opt, idx] = sort(y_opt);
x_opt = x_opt(:,idx);


%% Solve SDP relaxation
% Set optimization variable
X = sdpvar(n+1);
% Constraint on PSD matrix and constant multiplier to 1
F = [
    X >= 0,
    X(1) == 1,
];
% Constraint on diagonal terms
F = [F, 0 <= diag(X) <= ub^2];
% Constraint on off-diagonal terms (only holds for lb=-1 and ub=1)
F = [F, lb*ub <= X(logical(triu(true(n+1))-diag(true(n+1,1)))) <= ub^2];
% % Slow variant:
% % for i = 3:n+1
% %     F = [F, lb*ub <= X(2:i-1,i) <= ub^2];
% % end
% Perform optimization
obj =  trace(Q*X);
t2 = tic;
optimize(F, obj, sdpopt);
t2 = toc(t2);
% Calculate approximated value
y_app = value(trace(Q*X));


% Solutions
% disp('State approximation error: ');
% abs(x_opt(:,1) - x_app)
disp(' ');
disp(['Output approximaiton error [%]:      ', num2str((y_opt(1) - y_app)/abs(y_opt(1))*100), ' ']);
disp(['Time taken by full optimization [s]: ', num2str(t1), ' ']);
disp(['Time taken for SDP [s]:              ', num2str(t2), ' ']);


% C:\Users\dfzug\OneDrive\Desktop\Uni\ABB\src
% C:\Users\dfzug\scs-matlab

