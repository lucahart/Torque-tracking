%% My cheatsheet for YALMIP

% Before starting the program clear all previously initialized instances
% of YALMIP variables
yalmip('clear')

%% Variabe initializations
% A general variable is defined as follows, where n referrs to the row and
% m to the column size of the vector
n = 10; m = 1;
x = sdpvar(n,m);
% Scalar variables can also be defined in simpler ways
sdpvar a b c
% Quadradic matrices can be initialized with just one entry (here a 2x2
% matrix). By default they are initialized as symmetric matrices, if you
% want a full matrix, add the argument 'full'
P = sdpvar(3);
P1 = sdpvar(2,2,'full');
% Optain the values of the variables via the value-command
val = value(P);

% Most operators from MATLAB are overloaded in YALMIP. Thus, commands such
% as diag(x), sum(x), rand(x), length(x), trace(x), eye(x), and many more
% can be used


%% Defining constraints for an optimization problem
% Constraints can be defined in various ways. We will start with an empty
% constraint vector C and add subsequently show new possible additions for 
% vector constraints on x.
C = [];
% Lower and upper bounds
C = [C, x >= -20, x <= 10];
% Linear constraints
C = [C, sum(x) <= 50, [1 5 1 0 0 0 2 0 1 1]*x <= 100];
% Equality constraints
C = [C, x(6) == 5];
% Thre is also a possiblility for faster notation
C = [C, -5 <= x(7) <= 5];
% Lets add some more constraints for fun
for i = 1:7
    C = [C x(i)+x(i+1) <= x(i+2) + x(i+3)];
end
% Note that every time you use strict constraints a cute cat cries
% So far we are not able to handle non-convex constraints

% There are also possibilities for Matrix constraints That we will show
% with the constraint vector C1 for the matrix P.
Cp = [];
% Positive definite matrices
Cp = [Cp, P >= 0];
% Elementwise positive matrices
Cp = [Cp, P(:) >= 0];


%% Further definitions for the optimization problem
% The cost funcation J can be defined as follows
J = x'*x + norm(x,1);

% Setting options
options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

%% Solving the optimization problem
% Run the solver in the background via optimize
sol = optimize(C,J,options);

% Check for possible errors
if sol.problem == 0
    solution = value(x)
else
    sol.info
end

