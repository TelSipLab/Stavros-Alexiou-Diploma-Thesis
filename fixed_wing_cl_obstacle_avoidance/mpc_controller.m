function [ax, ay, ah, U0out, U_opt, J, exitflag, output] = ...
    mpc_controller(params, cs0, ref, A, B, Q, R, Rd, Hp, Hc, lb, ub, ...
        da_con, alpha, U0, Vw, u_prev, a_ref_prev, obstacles, obst_params)

    % dimensions
    nu = size(B,2); % nu = 3

    % cost function definition
    cost_fun = @(U) mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, ...
        Hp, Hc, ref, a_ref_prev, obstacles, obst_params);

    % mpc constraints definition
    constraints = @(U) mpc_constraints(U, u_prev, cs0, ref, A, B, Hp, Hc, ...
                       da_con, alpha, Vw, params, obstacles, obst_params);

    % fmincon settings
    options = optimoptions('fmincon', 'Algorithm', 'active-set', 'Display', 'off', ...
    'MaxIterations', 500, 'MaxFunctionEvaluations', 1e5, 'ConstraintTolerance', ...
                1e-4, 'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-8);

    % fmincon call (minimazation problem)
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
