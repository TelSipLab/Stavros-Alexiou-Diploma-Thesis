clc; clear; close all;

% UAV parameters
params = uav_params();

% Initial UAV conditions
x0 = 0; y0 = 0; h0 = 80;
Vg0 = 30; gamma0 = 0; psi0 = 0;

% s0_virtual = ref_state_circle(0);
% x0 = s0_virtual(1); y0 = s0_virtual(2); h0 = s0_virtual(3);
% dx0 = s0_virtual(4); dy0 = s0_virtual(5); dh0 = s0_virtual(6);
% Vg0 = sqrt(dx0^2 + dy0^2 + dh0^2);
% gamma0 = asin(dh0 / Vg0);
% psi0 = atan2(dy0, dx0);

% Initial state
s0 = [x0; y0; h0; Vg0; gamma0; psi0];

% UAV original control Variables (without di)
% ng = 1.3;           % g-load <-- Elevator
% phib = deg2rad(10); % banking angle <-- Rudder + Ailerons
% Th = 20;            % Thrust <-- Throttle
% control_vars = [ng phib Th];

% UAV double integrator control variables
% ax = 0.5; ay = 0.0; ah = 0.01;
% control_vars = [ax ay ah];

% UAV system dynamics
simulation_time = 120;
Tspan = [0 simulation_time];
[t, S] = ode45(@(t, s) uav_dynamics(t, s, params), Tspan, s0);

% extract uav state matrices
x = S(:, 1);
y = S(:, 2);
h = S(:, 3);
Vg = S(:, 4);
gamma = S(:, 5);
psi = S(:, 6);

% Generate reference trajectory
tt = linspace(0, simulation_time, length(t));  % tf is final simulation time
ref_circle = zeros(length(tt), 3);  % x, y, h

for i = 1:length(tt)
    s_ref = ref_state_circle(tt(i));
    ref_circle(i, :) = [s_ref(1), s_ref(2), s_ref(3)];
end

x_ref = ref_circle(:,1);
y_ref = ref_circle(:,2);
h_ref = ref_circle(:,3);

%% Trajectory Plots
figure;
plot3(x, y, h, 'b-', 'LineWidth', 1.5); hold on;
plot3(x_ref, y_ref, h_ref, 'r--', 'LineWidth', 1.5);

% Initial and final positions
plot3(x(1), y(1), h(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), h(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Labels
xlabel('x [m]'); ylabel('y [m]'); zlabel('h [m]');
title('UAV 3D Trajectory vs Reference Circle');
legend('UAV Trajectory', 'Reference Circle', 'Start', 'End');
grid on;
axis equal;

%% State Evolution Plots
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
title('Flight Path Angle \gamma');

% psi
subplot(6,1,6);
plot(t, rad2deg(S(:,6)), 'm', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\psi [deg]');
title('Heading Angle \psi');

sgtitle('UAV State Evolution');