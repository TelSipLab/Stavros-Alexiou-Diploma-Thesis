function [ax, ay, ah, U0out, J, exitflag, output] = ...
    mpc_controller(params, cs0, r, A, B, Q, R, Rd, Hp, Hc, lb, ub, U0, Vw, u_prev, a_ref_prev)

    % Dimensions
    nu = size(B,2); % nu = 3

    % cost function definition using only U
    cost_fun = @(U) mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, Hp, Hc, r, a_ref_prev);

    % fmincon settings
    options = optimoptions('fmincon', 'Display', 'none', ...
                                      'Algorithm', 'sqp');

    % mpc constraints
    CSmin = [-200; -200; 60; -30; -30; -3]; % x y h dx dy dh
    CSmax = [200; 200; 120; 30; 30; 3];
    Th_min = 0;

    constraints = @(U) mpc_constraints(U, cs0, A, B, Hp, Hc, ...
                                    CSmin, CSmax, Vw, params, Th_min);

    % minimazation problem
    [U_opt, J, exitflag, output] = fmincon(cost_fun, U0, ...
                                           [], [], [], [], ...
                                           lb, ub, constraints, options);
    % apply first control input
    u0 = U_opt(1:nu);
    ax = u0(1);
    ay = u0(2);
    ah = u0(3);

    % shift U0 and hold last output
    U0out = [U_opt(nu+1:end); U_opt(end-nu+1:end)];
end