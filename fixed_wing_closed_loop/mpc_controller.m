function [ax, ay, ah, U0out, exitflag, output] = ...
    mpc_controller(cs0, r, A, B, Q, R, Rd, Hp, Hc, lb, ub, U0, u_prev)

    % Dimensions
    nu = size(B,2); % nu = 3

    % cost function definition using only U
    cost_fun = @(U) mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, Hp, Hc, r);

    % fmincon settings
    options = optimoptions('fmincon', 'Display', 'none', ...
                                      'Algorithm', 'sqp');

    % minimazation problem
    [U_opt, ~, exitflag, output] = fmincon(cost_fun, U0, ...
                                           [], [], [], [], ...
                                           lb, ub, [], options);
    % apply first control input
    u0 = U_opt(1:nu);
    ax = u0(1);
    ay = u0(2);
    ah = u0(3);

    % shift U0 and hold last output
    U0out = [U_opt(nu+1:end); U_opt(end-nu+1:end)];
end
