function J = cost_function(U_opt, x0, A, B, Q, R, Hp, Hc, x_ref, Ts)
    nx = size(A, 1);
    X = zeros(nx, Hp+1);
    X(:, 1) = x0;
    J = 0;

    % Simulate forward using current U
    for k = 1:Hp
        if k <= Hc
            u = U_opt(k);
        else
            u = U_opt(Hc);  % hold last control input after Hc
        end
        
        % ode45
        tspan = [0 Ts];
        [t, x_ode] = ode45(@(t, x) double_integrator(t, x, u), tspan, X(:, k)); 
        X(:, k+1) = x_ode(end, :);  % Store the predicted state at time k+1
        
        % State error penalty
        e = X(:, k+1) - x_ref;
        J = J + e' * Q * e;  % Add state error penalty to cost function
        
        % Control effort penalty
        if k <= Hc
            J = J + u' * R * u;  % Add control effort penalty to cost function
        end
    end
end