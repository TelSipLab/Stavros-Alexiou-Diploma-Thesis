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
T = 30; % simulation time
Ts = 0.05; % sampling time
N = T/Ts; % total samples
tk = 0:Ts:T; % discrete time log

% prealloc
U_d = zeros(N,3);

% simulation loop
for k = 1:N

    % xronos metaksi dio digmatwn digmatolipsias
    tk1 = tk(k);
    tk2 = tk(k+1);
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
    if k == 1
        t_total = t_seg;
        S_total = S_seg;
    else
        t_total = [t_total; t_seg(2:end)];
        S_total = [S_total; S_seg(2:end,:)];
    end
    U_d(k,:) = u_k; % control log
end

% UAV data log
logs = uav_datalog(t_total, S_total, tk, U_d, @ref_state_circle, params);
E = logs.E;

% performance metrics
metrics = performance_metrics(t_total, Ts, logs);

% plots
% [~,~,~,K] = sf_controller(0, s0, ref_state_circle(0));
% plots_func(t, logs, K);