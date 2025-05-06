clc; clear; close all;

%% System Parameters

% MPC setup
T = 10;              % total simulation time
Ts = 0.1;            % sampling time
N = T / Ts;          % number of steps
Hp = 10;             % prediction horizon
Hc = 5;              % control horizon

% Discrete-time model (Forward Euler approximation)
A = [1 Ts; 0 1];
B = [0; Ts];

% Cost weights
Q = eye(2);          % penalize position + velocity error
R = 1;               % penalize control effort

% Constraints
umin = -1;
umax = 1;

% Bounds for optimization (fmincon)
lb = umin * ones(Hc, 1); % lower bound
ub = umax * ones(Hc, 1); % upper bound

% Initial and target state
x = [0; 0];          % initial state
x_ref = [10; 0];     % desired target

%% Prepare storage for results

X = zeros(2, N+1);   % state history
U = zeros(1, N);     % control history
X(:,1) = x;          % store initial state

%% MPC Loop

U0 = zeros(Hc, 1);  % Initial guess for the first iteration

for k = 1:N
    % Cost function definition using only U_opt
    cost_fun = @(U_opt) cost_function(U_opt, x, A, B, Q, R, Hp, Hc, x_ref, Ts);
    
    % Solve optimization problem
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    U_opt = fmincon(cost_fun, U0, [], [], [], [], lb, ub, [], options);
    
    % Apply first control input
    u = U_opt(1);
    U(k) = u;

    % Update state (with forward euler method)
    x = A * x + B * u;
    X(:, k+1) = x;

    % Shift U_opt for the next initial guess
    U0 = [U_opt(2:end); U_opt(end)];
end

%% Plots

time = 0:Ts:T;

figure;
subplot(3,1,1); % Position plot
plot(time, X(1,:), 'b', 'LineWidth', 2);
hold on;
yline(x_ref(1), '--r');  % Reference position line
xlabel('Time (s)');
ylabel('X_1 (Position)');
title('X_1(t)');
grid on;

subplot(3,1,2);  % Velocity plot
plot(time, X(2,:), 'g', 'LineWidth', 2);
hold on;
yline(x_ref(2), '--r');  % Reference velocity line
xlabel('Time (s)');
ylabel('X_2 (Velocity)');
title('X_2(t)');
grid on;

subplot(3,1,3);  % Control signal plot
stairs(time(1:end-1), U, 'k', 'LineWidth', 2);  % Control input plot
xlabel('Time (s)');
ylabel('U (Control Signal)');
title('U(t)');
yline(umin, '--r');  % Minimum control input
yline(umax, '--r');  % Maximum control input
grid on;