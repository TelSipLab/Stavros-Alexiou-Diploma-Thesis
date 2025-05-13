function J = cost_function(U_opt, x0, A, B, Q, R, Hp, Hc, x_target, Ts)
    states_number = size(A, 1);
    X = zeros(states_number, Hp+1);
    X(:, 1) = x0;
    J = 0;

    % Input Control
    U = ones(1,Hp);

    % Forward Euler Loop (creating the X matrix --> state predictions)
    for k = 1:Hp
     X(:, k+1) = A*X(:,k) + B*U(k);
    end

    % Simulate forward using current U
    for k = 1:Hp
        if k <= Hc
            u = U_opt(k);
        else
            u = U_opt(Hc);  % hold last control input after Hc
        end
       X(:, k+1) = A*X(:,k) + B*U(k);
    end
end
