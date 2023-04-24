% Quadratic Programming

% Clear previous simulations
yalmip('clear')
close all;

% Problem setup
x = [1 2 3 4 5 6]';
t = (0:0.02:2*pi)';
A = [sin(t) sin(2*t) sin(3*t) sin(4*t) sin(5*t) sin(6*t)];
err = (-4+8*rand(length(t),1));
err(100:115) = 30;
y = A*x+err;

% Optimization variables
xhat = sdpvar(6,1);
e = y-A*xhat;

% Minimize L1 norm
bound = sdpvar(length(e),1);
C = [-bound <= e <= bound];
optimize(C,sum(bound));
x_L1 = value(xhat);

% Minimize Linf norm
sdpvar bound;
C = [-bound <= e <= bound];
optimize(C,bound);
x_Linf = value(xhat);

% Minimize L2 norm
optimize([],e'*e);
x_L2 = value(xhat);

% Minimize L2 norm with L1 regularization
bound = sdpvar(length(e),1);
C = [-bound <= e <= bound];
J = e'*e + sum(bound); % equivalent to: J = e'*e + norm(e,1);->remove bound
optimize(C,J);
x_L2reg = value(xhat);

% The constrained set C and bound vector/scalar can be removed by directly
% using the correstponding cost function with the commands:
J = norm(e,1);
J = norm(e,inf);
J = norm(e,2) + norm(e,1);

% Plot the results
figure(1);
hold on;
plot(t,y);
plot(t,value(A*x));
plot(t,A*x_L1);
plot(t,A*x_Linf);
plot(t,A*x_L2);
plot(t,A*x_L2reg);
legend('y','x','x_{L1}','x_{Linf}','x_{L2}','x_{L2reg}');
hold off;
grid on;

