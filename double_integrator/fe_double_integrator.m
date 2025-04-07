clc; clear; close all;

% simulation parameters
T = 10;    % simulation time
Ts = 0.1; % sampling time
N = T/Ts;  % total number of steps

% initial conditions
x = zeros(2, N); % state vector (position & velocity)
x(:,1) = [0; 0]; % x1(0) = 0, x2(0) = 0

% input (u)
u = 1;

% Forward Euler Loop
for k = 1:N-1
    x(1, k+1) = x(1, k) + Ts * x(2, k);
    x(2, k+1) = x(2, k) + Ts * u;
end

% time stamps
t = 0:Ts:T-Ts;

% Figures
figure;
subplot(2,1,1);
plot(t, x(1,:), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position x_1');
title('Position vs Time');
grid on;

subplot(2,1,2);
plot(t, x(2,:), 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Velocity x_2');
title('Velocity vs Time');
grid on;