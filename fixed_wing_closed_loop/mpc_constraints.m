function [C, Ceq] = mpc_constraints(U, cs0, A, B, Hp, Hc, ...
                                    CSmin, CSmax, Vw, params, Th_min)
% mpc state prediction
CS = mpc_state_prediction(U, cs0, A, B, Hp, Hc); % 6 x Hp+1
CS_pred = CS(:, 2:end); % 6 x Hp

% state bounds 
cmax = CS_pred - CSmax; % 6 x Hp
cmin = CSmin - CS_pred; % 6 x Hp
C_state = [cmax(:); cmin(:)]; % 12*Hp x 1

% thrust bounds
nu = size(B,2); % nu = 3
U = reshape(U, nu, Hc); % U: 3x10 (reshape due to fmincon)
Th = zeros(1, Hp); % Th: 1x10
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
     [~, ~, Th(k), ~] = di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params);
end
C_thrust = (Th_min - Th(:)); % Hp x 1

% total mpc constraints vector
C = [C_state; C_thrust];
Ceq = [];
end