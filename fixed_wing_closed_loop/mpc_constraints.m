function [C, Ceq] = mpc_constraints(U, u_prev, cs0, ref, A, B, Hp, Hc, da_con, alpha, Vw, params)

con = uav_constraints();

% mpc state prediction
CS = mpc_state_prediction(U, cs0, A, B, Hp, Hc); % 6 x Hp+1
CS_pred = CS(:, 1:end-1); % 6 x Hp

% real controller output predictions
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3xHc (reshape due to fmincon)
phib = zeros(1, Hp);
ng = zeros(1, Hp);
Th = zeros(1, Hp);
Vg = zeros(1, Hp);
gamma = zeros(1, Hp);
for k = 1:Hp
     ax = U(1,k); 
     ay = U(2,k); 
     ah = U(3,k);
     dx = CS_pred(4,k);
     dy = CS_pred(5,k);
     dh = CS_pred(6,k);
     Vg(k) = sqrt(dx^2 + dy^2 + dh^2);
     gamma(k) = asin(dh / Vg(k));
     psi = atan2(dy, dx);
     [phib(k), ng(k), Th(k), ~] = di_mapping(ax, ay, ah, psi, gamma(k), Vg(k), Vw, params);
end

% mpc real controller output constraints
C_Th = [Th(:) - con.Th_max; con.Th_min - Th(:)];
C_ng = [ng(:) - con.ng_max; con.ng_min - ng(:)];
C_phib = [phib(:) - con.phib_max; con.phib_min - phib(:)];

% ground speed constraints
C_Vg = [Vg(:) - con.Vg_max; con.Vg_min - Vg(:)];

% flight path angle constraints
C_gamma = [gamma(:) - con.gamma_max; con.gamma_min - gamma(:)];

% da constraints
da_min = da_con(1); da_max = da_con(2);
dU = zeros(nu, Hc);
dU(:,1) = U(:,1) - u_prev;
dU(:,2:end) = U(:,2:end) - U(:,1:end-1);
C_da = [dU(:) - da_max; da_min - dU(:)];

% contractive mpc constraints
[C_Vx, C_Vy, C_Vh] = mpc_contractive_constraints(cs0, ref, CS, alpha);

% total mpc constraints vector
C = [C_Th; C_ng; C_phib; C_Vg; C_gamma; C_da; C_Vx; C_Vy; C_Vh];
Ceq = [];

end