function J = cost_func_mpc(U, s0, A, B, Q, R, Hp, Hc, r)

Umat = reshape(U, 3, Hc);
nx = length(s0);
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
tracking_cost = sum( vecnorm((Q^(1/2))*E,2,1).^2 );
control_cost = sum( vecnorm((R^(1/2))*Umat,2,1).^2 );
J = tracking_cost + control_cost;

end
