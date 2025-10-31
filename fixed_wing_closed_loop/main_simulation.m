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

%% Main Simulation
% Simulation parameters
T = 60; % simulation time
Ts = 0.05; % sampling time
N = T/Ts; % total samples
tk = 0:Ts:T; % discrete time log

% simulation loop
t = []; S = [];
U_d = zeros(N,3);
for k = 0:N-1

    % xronos metaksi dio digmatwn digmatolipsias
    tk1 = k*Ts;  
    tk2 = (k+1)*Ts;
    Tspan = [tk1 tk2];

    % digmatolipsia reference
    r_k = ref_state_circle(tk1);

    % controller call
    [ax,ay,ah, ~] = sf_controller(tk1, s0, r_k);
    u_k = [ax ay ah];

    % uav dynamics call (tmimatiki oloklirosi)
    ode_fun = @(t,s) uav_dynamics(t, s, u_k, params);
    [t_seg, S_seg] = ode45(ode_fun, Tspan, s0);

    % final state values become next initial conditions
    s0 = S_seg(end,:).';

    % logs
    t_total = [t; t_seg]; % times log (countinuous)
    S_total = [S; S_seg]; % state log (countinuous)
    U_d(k+1,:) = u_k; % control log (discrete)
end

% UAV data log
logs = uav_datalog(t_total, S_total, tk, U_d, @ref_state_circle, params);
E = logs.E;
ref = logs.ref;
x = logs.x; y = logs.y; h = logs.h;

% plots
[~,~,~,K] = sf_controller(0, s0, ref_state_circle(0));
plots_func(t, logs, K);

%% performance metrics
% P = S(:,1:3);
% R = ref(:,1:3);
% metrics = performance_metrics(t, P, R, E, U);
% disp(metrics.table_axes)
% disp(metrics.table_errors)
% disp(metrics.table_control)