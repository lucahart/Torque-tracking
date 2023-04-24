%% Semidefinite Programming
yalmip('clear');

% Define dynamics of autonomous continuous time system x_dot = A*x
A = [-1 2 0;-3 -4 1; 0 0 -2];
% Define variable to optimize
P = sdpvar(3,3);
% Set constraints, including positive definiteness of P, negative
% definiteness of the lyapunov function's V(x) = x'*x derivative, and set
% the trace unequal 0 to avoid a trivial all-zero solution
F = [P >= 0, A'*P + P*A <= 0, trace(P) == 1];

% Run the optimization without an objective, as we just want to find some
% feasible solution
optimize(F);
% Output value of P
% value(P)

% Robust problems
P = sdpvar(3);
F = [P >= 0];
for i = 1:20
    Ai = A + rand(3);
    F = [F, Ai*P + P*Ai <= 0];
end
optimize(F, trace(P));

%% Sum of Squares
yalmip('clear');

% Introduce a Polynomial
sdpvar x y;
p = (1+x)^4 + (1-y)^2;
% 
v = monolist([x y], degree(p)/2);
Q = sdpvar(length(v));
p_sos = v'*Q*v;
%
F = [coefficients(p-p_sos,[x y]) == 0, Q >= 0];
optimize(F)

% % Fast way with built-in sos function
% sdpvar x y;
% p = (1+x)^4 + (1-y)^2;
% F = sos(p);
% solvesos(F);
% % Optian sos decomposition
% h = sosd(F);
% sdisplay(h)



%% Designing Polynomials
yalmip('clear');

% Value definitions
x0 = -1; x1 = 0; x2 = 1; y0 = 0; y1 = 1; y2 = 0; n = 9;

% Simple example
x = sdpvar(1);
% Create polynomial p, with coefficients a, and v is the basis in p = a'*v
[p,a,v] = polynomial(x,n);
% Set model constraints
Model = [
    replace(p,x,x0) == y0,
    replace(p,x,x1) >= y1,
    replace(p,x,x2) == y2
];
optimize(Model, abs(a(9)));

% Plotting
xv = linspace(x0,x2,100);
yv = polyval(fliplr(value(a')),xv);
close 1;
figure(1); grid on; hold on;
plot(xv,yv);

% Extension with derivatives
dp = jacobian(p,x);
dp2 = hessian(p,x);
dModel = [
    Model,
    replace(dp,x,x0) == 0,
    replace(dp,x,x2) == 0,
    replace(dp2,x,x1) >= 0
];
optimize(dModel);
yv = polyval(fliplr(value(a')),xv);
plot(xv,yv);

% Extension with integrals
optimize(dModel, int(p^2,x,-1,1));
yv = polyval(fliplr(value(a')),xv);
plot(xv,yv);

% Useful commands
sdisplay(p) % Displays the polynomial p
[a, v] = coefficients(p, x); % Gives the coefficients and basis of p


