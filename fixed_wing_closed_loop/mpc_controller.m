function [ax, ay, ah, U0_next, exitflag, output] = ...
    mpc_controller(s0, r, A, B, Q, R, Hp, Hc, lb, ub, U0)

    % Dimensions
    nu = size(B,2);

    % cost function definition using only U
    cost_fun = @(U) mpc_cost_func(U, s0, A, B, Q, R, Hp, Hc, r);

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
    U0_next = [U_opt(nu+1:end); U_opt(end-nu+1:end)];
end
