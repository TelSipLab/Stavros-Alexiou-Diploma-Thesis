%% paper --> integrated optimal formation control
clc; clear;

%% Epeksigisi parametrwn kai statheres
% x,y,h --> drone position
% dx,dy,dh --> drone speed
% Vg --> ground speed
% gamma --> fligth path angle
% psi --> heading angle
% Th --> engine thrust
% Dg --> Drag
% m --> mass
% ga --> accel. due to grav.
% ng --> g-load
% Lf --> lift
% phib --> banking angle

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

% UAV Control Variables
ng = 1.3;          % g-load <-- Elevator
phib = deg2rad(10);  % banking angle <-- Rudder + Ailerons
Th = 20;           % Thrust <-- Throttle
control_vars = [ng phib Th];

% ODE
simulation_time = 60;
Tspan = [0 simulation_time];
[t, S] = ode45(@(t, s) uav_dynamics(t, s, params, control_vars), Tspan, s0);

% extract drone position every step
x = S(:, 1);
y = S(:, 2);
h = S(:, 3);

% trajectory plot
figure;
plot3(x, y, h, 'b-', 'LineWidth', 1.5); hold on;

% initial & final drone position
plot3(x(1), y(1), h(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), h(end), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% labels
xlabel('x [m]');
ylabel('y [m]');
zlabel('h [m]');
title('UAV 3D Trajectory');
grid on;
legend('Drone Trajectory', 'Initial Position', 'Final Position');