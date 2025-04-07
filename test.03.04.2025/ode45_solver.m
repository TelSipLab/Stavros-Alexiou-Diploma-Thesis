clc; clear; close all;

% arxikes sinthikes
x0 = [0; 0];

% xroniko diastima
tspan = [0 10];

% control signal (input)
u = 1;

% solve using ode45
[t, x] = ode45(@(t, x) double_integrator(t, x, u), tspan, x0);

% figures
figure;
subplot(2,1,1);
plot(t, x(:,1), 'b', 'LineWidth', 2);
xlabel('time (s)'); ylabel('position x_1');
title('position vs time');
grid on;

subplot(2,1,2);
plot(t, x(:,2), 'r', 'LineWidth', 2);
xlabel('time (s)'); ylabel('velocity x_2');
title('velocity vs time');
grid on;
