function [ax, ay, ah, U0out, J, exitflag, output] = ...
    mpc_controller(cs0, r, A, B, Q, R, Rd, Hp, Hc, lb, ub, U0, u_prev, a_ref_prev)

    % Dimensions
    nu = size(B,2); % nu = 3

    % cost function definition using only U
    cost_fun = @(U) mpc_cost_func(U, u_prev, cs0, A, B, Q, R, Rd, Hp, Hc, r, a_ref_prev);

    % fmincon settings
    options = optimoptions('fmincon', 'Display', 'none', ...
                                      'Algorithm', 'sqp');

    % state constraints
    Smin = [-500; -500; 60; -30; -30; -3];
    Smax = [500; 500; 120; 30; 30; 3];
    state_constraints = @(U) mpc_state_constraints(U, cs0, A, B, Hp, Hc, Smin, Smax);

    % minimazation problem
    [U_opt, J, exitflag, output] = fmincon(cost_fun, U0, ...
                                           [], [], [], [], ...
                                           lb, ub, state_constraints, options);
    % apply first control input
    u0 = U_opt(1:nu);
    ax = u0(1);
    ay = u0(2);
    ah = u0(3);

    % shift U0 and hold last output
    U0out = [U_opt(nu+1:end); U_opt(end-nu+1:end)];
end
