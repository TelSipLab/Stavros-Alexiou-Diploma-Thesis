function J = mpc_cost_func(U, s0, A, B, Q, R, Hp, Hc, ref)

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

E = S(:,2:end) - ref; % 6 x Hp

% tracking cost
Jx = Q(1,1) * (E(1,:)*E(1,:).');
Jy = Q(2,2) * (E(2,:)*E(2,:).');
Jh = Q(3,3) * (E(3,:)*E(3,:).');
Jdx = Q(4,4) * (E(4,:)*E(4,:).');
Jdy = Q(5,5) * (E(5,:)*E(5,:).');
Jdh = Q(6,6) * (E(6,:)*E(6,:).');
tracking_cost = Jx + Jy + Jh + Jdx + Jdy + Jdh;
% tracking_cost = sum(diag(E.'*Q*E));

% control cost
Jax = R(1,1) * (U(1,:)*U(1,:).');
Jay = R(2,2) * (U(2,:)*U(2,:).');
Jah = R(3,3) * (U(3,:)*U(3,:).');
control_cost = Jax + Jay + Jah;
% control_cost = sum(diag(U.'*R*U));

J = tracking_cost + control_cost;

end