clc; clear;

% UAV parameters
params = uav_params();

% Initial conditions
x0 = 100;
y0 = 200;
h0 = 80;
Vg0 = 40;
gamma0 = 0;
psi0 = 0;

% Initial state
s0 = [x0; y0; h0; Vg0; gamma0; psi0];

% UAV Original Control Variables
%ng = 1.3;           % g-load <-- Elevator
%phib = deg2rad(10); % banking angle <-- Rudder + Ailerons
%Th = 20;            % Thrust <-- Throttle
%control_vars = [ng phib Th];

% Double integrator control variables
ax = 0.1;
ay = 0;
ah = 0.05;
control_vars = [ax ay ah];

% ODE
simulation_time = 60;
Tspan = [0 simulation_time];
[t, S] = ode45(@(t, s) uav_dynamics(t, s, params, control_vars), Tspan, s0);

% extract drone position every step
x = S(:, 1);
y = S(:, 2);
h = S(:, 3);
Vg = S(:, 4);
gamma = S(:, 5);
psi = S(:, 6);

%% PLOTS

% trajectory plot
figure;
plot3(x, y, h, 'b-', 'LineWidth', 1.5); hold on;

% initial & final drone position
plot3(x(1), y(1), h(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), h(end), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% trajectory labels
xlabel('x [m]');
ylabel('y [m]');
zlabel('h [m]');
title('UAV 3D Trajectory');
grid on;
legend('Drone Trajectory', 'Initial Position', 'Final Position');

figure;

% x
subplot(6,1,1);
plot(t, S(:,1), 'b', 'LineWidth', 1.5);
grid on;
ylabel('x [m]');
title('Position x');

% y
subplot(6,1,2);
plot(t, S(:,2), 'b', 'LineWidth', 1.5);
grid on;
ylabel('y [m]');
title('Position y');

% h
subplot(6,1,3);
plot(t, S(:,3), 'b', 'LineWidth', 1.5);
grid on;
ylabel('h [m]');
title('Altitude h');

% Vg
subplot(6,1,4);
plot(t, S(:,4), 'c', 'LineWidth', 1.5);
grid on;
ylabel('V_g [m/s]');
title('Ground Speed');

% gamma 
subplot(6,1,5);
plot(t, rad2deg(S(:,5)), 'm', 'LineWidth', 1.5);
grid on;
ylabel('\gamma [deg]');
title('Flight Path Angle');

% psi
subplot(6,1,6);
plot(t, rad2deg(S(:,6)), 'm', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\psi [deg]');
title('Heading Angle');

sgtitle('UAV State Evolution');