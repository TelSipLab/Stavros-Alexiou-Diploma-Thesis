function J = mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, Hp, Hc, ref)

% prealloc state vector
nx = size(A,1); % nx = 6
S = zeros(nx, Hp+1); % S: 6x11

% reshape control vector
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3x10

% dU calculation
dU = zeros(nu, Hc); % dU: 3x10
dU(:,1) = U(:,1) - u_prev;
dU(:,2:end) = U(:,2:end) - U(:,1:end-1);

% state calculation
S(:,1) = cs0;
for k = 1:Hp
    if k <= Hc
        u_k = U(:,k);
    else
        u_k = U(:,Hc);
    end

    S(:,k+1) = A*S(:,k) + B*u_k;
end

% error calculation
E = S(:, 2:end) - ref(1:6,:); % 6x10

% tracking cost
Jx = E(1,:) * Q(1,1) * E(1,:).';
Jy = E(2,:) * Q(2,2) * E(2,:).';
Jh = E(3,:) * Q(3,3) * E(3,:).';
Jdx = E(4,:) * Q(4,4) * E(4,:).';
Jdy = E(5,:) * Q(5,5) * E(5,:).';
Jdh = E(6,:) * Q(6,6) * E(6,:).';
tracking_cost = Jx + Jy + Jh + Jdx + Jdy + Jdh;
% tracking_cost = sum(diag(E*Q*E.'));

% control cost
Jax = (U(1,:) - ref(7,:)) * R(1,1) * (U(1,:) - ref(7,:)).';
Jay = (U(2,:) - ref(8,:)) * R(2,2) * (U(2,:) - ref(8,:)).';
Jah = (U(3,:) - ref(9,:)) * R(3,3) * (U(3,:) - ref(9,:)).';
control_cost = Jax + Jay + Jah;
% control_cost = sum(diag(U*R*U.'));

% dU_cost
Jdax = dU(1,:) * Rd(1,1) * dU(1,:).';
Jday = dU(2,:) * Rd(2,2) * dU(2,:).';
Jdah = dU(3,:) * Rd(3,3) * dU(3,:).';
dU_cost = Jdax + Jday + Jdah;
% dU_cost = sum(diag(dU*Rd*dU.'));

% total cost of mpc_cost_func
J = tracking_cost + control_cost + dU_cost;

end