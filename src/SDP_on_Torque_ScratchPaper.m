

n_x = 4;
n_x2 = 2;
m = 3;

Gamma_s = zeros(2*N,n_x);
for i = 0:N-1
    Gamma_s(2*i + 1:2*(i+1), :) = A_sim(1:2,:)*A_sim^i;
end

Upsilon_s = zeros(2*N,3*N);
for i = 0:N-1
    for j = 0:i
        if i == j
            Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = ctrl.B_2;
        else
            Upsilon_s(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A_sim(1:2,:)*A_sim^(i-j-1)*B_sim;
        end
    end
end

Gamma_r = zeros(2*N,n_x);
for i = 0:N-1
    Gamma_r(2*i + 1:2*(i+1), :) = A_sim(3:4,:)*A_sim^i;
end

Upsilon_r = zeros(2*N,3*N);
for i = 0:N-1
    for j = 0:i-1
        Upsilon_r(2*i+1:2*(i+1),3*j+1:3*(j+1)) = A_sim(3:4,:)*A_sim^(i-j-1)*B_sim;
    end
end

Zeta = [0 1; -1 0];

S = eye(3*N) - [
    zeros(3,3*N);
    eye(3*(N-1)) zeros(3*(N-1),3)
];

E = [eye(3); zeros(3*(N-1),3)];
% Zeta = zeros(2*N);
% for i = 0:N-1
%     Zeta(2*i+1:2*(i+1),2*i+1:2*(i+1)) = [0 1; -1 0];
% end

Q = @(x,i) [
    x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, x'*Gamma_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:);
    Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Gamma_s(2*i+1:2*(i+1),:)*x, Upsilon_r(2*i+1:2*(i+1),:)'*Zeta*Upsilon_s(2*i+1:2*(i+1),:)
];

W = @(x,i) [
    x'*Gamma_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, x'*Gamma_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:);
    Upsilon_s(2*i+1:2*(i+1),:)'*Gamma_s(2*i+1:2*(i+1),:)*x, Upsilon_s(2*i+1:2*(i+1),:)'*Upsilon_s(2*i+1:2*(i+1),:)
];

Y = @(u_prev) [
    u_prev'*E'*E*u_prev, -u_prev'*E'*S;
    -S'*E*u_prev, S'*S
];

X = sdpvar(3*N+1);

% Constraint on PSD matrix and constant multiplier to 1
F = [
    X >= 0,
    X(1) == 1,
];
% Constraint on diagonal terms
F = [F, 0 <= diag(X) <= ub^2];
% Constraint on off-diagonal terms (only holds for lb=-1 and ub=1)
F = [F, lb*ub <= X(logical(triu(true(3*N+1))-diag(true(3*N+1,1)))) <= ub^2];

% Perform optimization
J = 0;%ctrl.lam_u*trace(Y(u_prev)*X);
for i = 0:1%N-1
    J = J + ctrl.lam_T*(T_ref - ctrl.T_factor*trace(Q(x,i)*X))^2 ...
        + (1-ctrl.lam_T)*(Psi_ref - trace(W(x,i)*X))^2;
end
t2 = tic;
optimize(F, J, sdpopt);
t2 = toc(t2);
% Calculate approximated value
% y_app = value(norm(T_ref-ctrl.T_factor*trace(Q(x)*X),2));



