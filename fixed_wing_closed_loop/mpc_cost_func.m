function J = mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, Hp, Hc, ref, a_ref_prev)

% reshape control vector
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3x10

% dU calculation
dU = zeros(nu, Hc); % dU: 3x10
dU(:,1) = U(:,1) - u_prev;
dU(:,2:end) = U(:,2:end) - U(:,1:end-1);

% dUref calculation
Uref = ref(7:9,:); % 3x10
dUref = zeros(3, Hc); % 3x10
dUref(:,1) = Uref(:,1) - a_ref_prev;
dUref(:,2:end) = Uref(:,2:end) - Uref(:,1:end-1);

% mpc state prediction
S = mpc_state_prediction(U, cs0, A, B, Hp, Hc);

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

% control cost
Jax = (U(1,:) - ref(7,:)) * R(1,1) * (U(1,:) - ref(7,:)).';
Jay = (U(2,:) - ref(8,:)) * R(2,2) * (U(2,:) - ref(8,:)).';
Jah = (U(3,:) - ref(9,:)) * R(3,3) * (U(3,:) - ref(9,:)).';
control_cost = Jax + Jay + Jah;

% dU_cost
Jdax = (dU(1,:) - dUref(1,:)) * Rd(1,1) * (dU(1,:) - dUref(1,:)).';
Jday = (dU(2,:) - dUref(2,:)) * Rd(2,2) * (dU(2,:) - dUref(2,:)).';
Jdah = (dU(3,:) - dUref(3,:)) * Rd(3,3) * (dU(3,:) - dUref(3,:)).';
dU_cost = Jdax + Jday + Jdah;

% total cost of mpc_cost_func
J = tracking_cost + control_cost + dU_cost;

end