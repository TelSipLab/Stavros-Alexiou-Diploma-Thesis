clc; clear; close all;

Ts = 0.1;
tspan = [0 Ts];
x0 = [0; 0];
u = [1 2 3 0.5 0.2];

% State matrix for each step
X = zeros(length(x0), length(u)+1);
X(:,1) = x0;
x_current = x0;
for i = 1:length(u)
    [t, x] = ode45(@(t, x) di(t, x, u(i)), tspan, x_current);
    x_current = x(end, :)';
    X(:, i+1) = x_current;
end

% Figures
total_time = 0:Ts:Ts*length(u);
plot(total_time, X(1, :), 'b-o', 'DisplayName', 'x1 (position)');
hold on;
plot(total_time, X(2, :), 'r-o', 'DisplayName', 'x2 (velocity)');
title('State Evolution Over Time');
xlabel('Time (s)');
ylabel('States');
legend;
grid on;