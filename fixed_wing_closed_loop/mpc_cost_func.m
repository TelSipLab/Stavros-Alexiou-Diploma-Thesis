function J = mpc_cost_func(U, s0, A, B, Q, R, Hp, Hc, r)

nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc);

nx = size(A,1); % nx = 6
S = zeros(nx, Hp+1);

S(:,1) = s0;
for k = 1:Hp
    if k <= Hc
        u_k = U(:,k);
    else
        u_k = U(:,Hc);
    end

    S(:,k+1) = A*S(:,k) + B*u_k;
end

E = S(:,2:end) - r;
tracking_cost = sum(diag(E.'*Q*E));
control_cost = sum(diag(U.'*R*U));
J = tracking_cost + control_cost;

end