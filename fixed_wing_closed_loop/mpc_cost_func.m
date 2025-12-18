function J = mpc_cost_func(U, s0, A, B, Q, R, Hp, Hc, r)

nu = size(B,2);
Umat = reshape(U, nu, Hc);

nx = size(A,1);
S = zeros(nx, Hp+1);

S(:,1) = s0;

for k = 1:Hp
    if k <= Hc
        u_k = Umat(:,k);
    else
        u_k = Umat(:,Hc);
    end

    S(:,k+1) = A*S(:,k) + B*u_k;
end

E = S(:,2:end) - r;
tracking_cost = sum(diag(E.'*Q*E));
control_cost  = sum(diag(Umat.'*R*Umat));
J = tracking_cost + control_cost;

end