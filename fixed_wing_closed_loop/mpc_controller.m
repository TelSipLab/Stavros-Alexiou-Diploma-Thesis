function [ax, ay, ah] = mpc_controller(s0, r_k, mpc_setup); 

% cost_fun definition using only U for fmincon use
 cost_fun_mpc = @(U) cost_func_mpc(U, A, B, Q, R, Hp, Hc, X(:,k), x_target);               

 % optimization problem solving
 [U_opt, J, exit_flag, output] = ... 
                fmincon(cost_fun_mpc, U0, [], [], [], [], lb, ub, []);
 fprintf('Step %d: Exit Flag = %d, Iterations = %d, J = %.4f\n', ... 
                k, exit_flag, output.iterations, J);

 % update control signal storage (first control input)
 U(k) = U_opt(1);

 % apply first control input to real system
 [t, x] = ode45(@(t, x) di(t, x, u), [0 Ts], X(:,k));
 X(:, k+1) = x(end,:)'; % update state matrix storage

 % shift U_opt for the next initial U (U0)
 U0 = [U_opt(2:end); U_opt(end)];

 end