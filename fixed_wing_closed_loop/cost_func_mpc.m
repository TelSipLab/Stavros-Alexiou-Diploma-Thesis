function J = cost_func_mpc(U, s0, r, A, B, Q, R, Hp, Hc)

    nx = numel(s0); % nx = 6
    nu = size(B, 2); % nu = 3
    U = reshape(U, nu, Hc);  % (3*Hc)x1 --> 3xHc

    % State predictions
    S = zeros(nx, Hp+1);
    S(:,1) = s0;
    for k = 1:Hp
        if k <= Hc
            S(:, k+1) = A*S(:, k) + U(:, k);
        else
            S(:, k+1) = A*S(:, k) + U(:, Hc);
        end
    end

    % Total Cost
    E = S(:,2:end) - r;
    tracking_error = trace(E.' * Q * E);
    control_effort = trace(U.' * R * U);
    J = tracking_error + control_effort;

end