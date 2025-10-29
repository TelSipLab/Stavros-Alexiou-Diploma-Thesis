clc; clear; close all;

% UAV parameters
params = uav_params();

% Initial UAV conditions
x0 = -10; 
y0 = 20; 
h0 = 60;
Vg0 = 30; 
gamma0 = 0; 
psi0 = 0;
s0 = [x0; y0; h0; Vg0; gamma0; psi0];

% Simulate UAV Dynamics
simulation_time = 60;
Tspan = [0 simulation_time];
[t, S] = ode45(@(t, s) uav_dynamics(t, s, params), Tspan, s0);

% UAV data log
logs = uav_datalog(t, S, params, @ref_state_circle, @sf_controller);
E = logs.E;
U = logs.U;
ref = logs.ref;
x = logs.x; y = logs.y; h = logs.h;

% plots
[~,~,~,K] = sf_controller(0, s0, ref_state_circle(0));
plots_func(t, logs, K);

% performance metrics
P = S(:,1:3);
R = ref(:,1:3);
metrics = performance_metrics(t, P, R, E, U);
disp(metrics.table_axes)
disp(metrics.table_errors)
disp(metrics.table_control)