function [C, Ceq] = mpc_constraints(U, cs0, A, B, Hp, Hc, Vw, params)

% mpc control state constraints
CSmin = [-200; -200; 60; -30; -30; -3];
CSmax = [200; 200; 120; 30; 30; 3];

% mpc state prediction
CS = mpc_state_prediction(U, cs0, A, B, Hp, Hc); % 6 x Hp+1
CS_pred = CS(:, 2:end); % 6 x Hp

% state bounds 
cmax = CS_pred - CSmax; % 6 x Hp
cmin = CSmin - CS_pred; % 6 x Hp
C_state = [cmax(:); cmin(:)]; % 12*Hp x 1

% real controller output predictions
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3xHc (reshape due to fmincon)
phib = zeros(1, Hp);
ng = zeros(1, Hp);
Th = zeros(1, Hp);
for k = 1:Hp
     ax = U(1,k); 
     ay = U(2,k); 
     ah = U(3,k);
     dx = CS_pred(4,k);
     dy = CS_pred(5,k);
     dh = CS_pred(6,k);
     Vg = sqrt(dx^2 + dy^2 + dh^2);
     gamma = asin(dh / Vg);
     psi = atan2(dy, dx);
     [phib(k), ng(k), Th(k), ~] = di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params);
end

% mpc real controller output constraints
con = uav_constraints();

% mpc real controller output constraints vector
C_Th = [Th(:) - con.Th_max; con.Th_min - Th(:)];
C_ng = [ng(:) - con.ng_max; con.ng_min - ng(:)];
C_phib = [phib(:) - con.phib_max; con.phib_min - phib(:)];
C_control = [C_Th; C_ng; C_phib];

% total mpc constraints vector
C = [C_state; C_control];
Ceq = [];

end