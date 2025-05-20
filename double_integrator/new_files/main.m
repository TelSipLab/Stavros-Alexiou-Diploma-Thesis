%% Clear all
clc; clear; close all;

%% System Parameters

% MPC Setup
T = 20;
Ts = 0.1;            
N = T / Ts;          
Hp = 10;             
Hc = 5;            

% Discrete-time model (Forward Euler Approximation)
A = [1 Ts; 0 1];
B = [0; Ts];

% Cost weights
Q = eye(2);          
R = 1;               

% Constraints
umin = -1;
umax = 1;

% Bounds for optimization (fmincon)
lb = umin * ones(Hc, 1);
ub = umax * ones(Hc, 1);

% Initial and target state
x0 = [0; 0];
x_target = [10; 0];
        
%% MPC Loop

X = zeros(2, N+1); % state matrix storage
U = zeros(1, N); % control signal storage
U0 = zeros(Hc, 1);  
X(:,1) = x0;

for k = 1:N

 % cost_fun definition using only U for fmincon use
 cost_fun = @(U) cost_func(U, A, B, Q, R, Hp, Hc, X(:,k), x_target);

 % optimization problem options
 options1 = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
 options2 = optimoptions('fmincon', 'Display', 'iter-detailed', 'Algorithm', 'sqp','PlotFcn', ...
     {@optimplotfval});               

 % optimization problem solving
 %U_opt = fmincon(cost_fun, U0, [], [], [], [], lb, ub, [], options1);
 [U_opt, J, exit_flag, output] = fmincon(cost_fun, U0, [], [], [], [], lb, ub, [], options1);
 fprintf('Step %d: Exit Flag = %d, Iterations = %d, J = %.4f\n', k, exit_flag, output.iterations, J);

 % first control input
 u = U_opt(1);
 U(k) = u; % update control signal storage

 % apply first control input to real system
 [t, x] = ode45(@(t, x) di(t, x, u), [0 Ts], X(:,k));
 X(:, k+1) = x(end,:)'; % update state matrix storage

 % shift U_opt for the next initial U (U0)
 U0 = [U_opt(2:end); U_opt(end)];

end

%% Plots

time = 0:Ts:T;

figure;
subplot(3,1,1);
plot(time, X(1,:), 'b', 'LineWidth', 2);
hold on;
yline(x_target(1), '--r');
xlabel('Time (s)');
ylabel('X_1 (Position)');
title('X_1(t)');
grid on;

subplot(3,1,2);
plot(time, X(2,:), 'g', 'LineWidth', 2);
hold on;
yline(x_target(2), '--r');
xlabel('Time (s)');
ylabel('X_2 (Velocity)');
title('X_2(t)');
grid on;

subplot(3,1,3);
stairs(time(1:end-1), U, 'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('U (Control Signal)');
title('U(t)');
yline(umin, '--r');
yline(umax, '--r');
grid on;