function K = lqr_gain_di()

    % double integrator model
    A_base = [0 1; 0 0]; % 2x2
    B_base = [0; 1]; % 2x1
    I3 = eye(3); % identity dew to 3D model (x, y, h)
    A = kron(A_base, I3); % 6x6
    B = kron(B_base, I3); % 6x3

    % lqr weights
    Q = diag([10 10 10 1 1 1]); % state error
    R = 0.1*eye(3); % control effort

    % compute gain matrix
    K = lqr(A, B, Q, R);  % Output: 3x6
end