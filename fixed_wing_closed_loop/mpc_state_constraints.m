function [C, Ceq] = mpc_state_constraints(U, cs0, A, B, Hp, Hc, Smin, Smax)

S = mpc_state_prediction(U, cs0, A, B, Hp, Hc); % S: 6x11
S_pred = S(:, 2:end); % S_pred: 6x10
cmax = S_pred - Smax; % cmax: 6x10
cmin = Smin - S_pred; % cmin: 6x10
C = [cmax(:); cmin(:)]; % C: 120x1
Ceq = [];

end