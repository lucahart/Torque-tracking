function [u_hat] = run_sdp(ctrl, x, u_prev, ref, method)

T_ref = ref(1);
Psi_ref = ref(2);
N = ctrl.N;
sdp = ctrl.sdp;

X = sdp.X;

% Cost function
J = sdp.J0;
% Add switching penalty to cost term
J = J + ctrl.lam_u*norm(trace(sdp.Y0a(u_prev)*X),1);
J = J + ctrl.lam_u*norm(trace(sdp.Y0b(u_prev)*X),1);
J = J + ctrl.lam_u*norm(trace(sdp.Y0c(u_prev)*X),1);

% Add tracking penalties to cost term
D = [];
for i = 0:N-1
    D = [
        D; 
        sqrt(ctrl.lam_T)*(T_ref - ctrl.T_factor*trace(sdp.Q(x,i)*X));
        sqrt(1-ctrl.lam_T)*(Psi_ref^2 - trace(sdp.W(x,i)*X))
    ];
end
J = J + norm(D,2)^2;
% Perform optimization
assign(X,[1; ctrl.U_ed]*[1 ctrl.U_ed']);
t2 = tic;
optimize(sdp.F, J, sdp.sdpopt);
t2 = toc(t2)

% Calculate approximated value
[V, Lambda] = eig(value(X));
v1 = V(:,end)*sqrt(Lambda(end));
v1 = sign(v1(1))*v1(2:end);

% round first column
u_hat1 = sign(value(X(1)))*round(value(X(2:end,1)));
% round sqrt of diagonal
u_hat2 = sign(value(X(1)))*round(sqrt(diag(value(X(2:end,2:end)))));
% round normally
u_hat3 = round(v1);
% round with equal distribution over all numbers
u_hat4 = zeros(3*N,1);
u_hat4(v1 > 1/3) = 1;
u_hat4(v1 < -1/3) = -1;

u_hat = [u_hat1 u_hat2 u_hat3 u_hat4];

end