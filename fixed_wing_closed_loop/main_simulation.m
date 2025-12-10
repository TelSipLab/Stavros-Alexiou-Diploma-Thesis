clc; clear; close all;

% UAV parameters
params = uav_params();

% Initial UAV conditions
x0 = -2;
y0 = -7; 
h0 = 60;
Vg0 = 30; 
gamma0 = 0; 
psi0 = 0;
s0 = [x0; y0; h0; Vg0; gamma0; psi0];

% Simulation Parameteres
T = 60; % simulation time
Ts = 0.05; % sampling time
N = T/Ts; % total samples
tk = 0:Ts:T; % discrete time log

%% PID Setup

% previus e errors
s_ref = ref_state_circle(0);
pid_errors.ex_prev = s_ref(1) - x0; 
pid_errors.ey_prev = s_ref(2) - y0; 
pid_errors.eh_prev = s_ref(3) - h0;
% due to: ed[k] = (e[k]-e[k-1])/Ts)

% previus ei errors
pid_errors.ex_int = 0; 
pid_errors.ey_int = 0; 
pid_errors.eh_int = 0;

%% MPC Setup

% Discrete-time 3D model (Forward Euler Approximation)
A = [1 0 0 Ts 0 0; 
     0 1 0 0 Ts 0;
     0 0 1 0 0 Ts;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

B = [0 0 0;
     0 0 0;
     0 0 0;
     Ts 0 0;
     0 Ts 0;
     0 0 Ts];

% prediction & control horizon
Hp = 10; Hc = 5;

% Cost weights for cost function
Q = eye(2); R = 1;               

% Constraints & bounds for optimization (fmincon)
umin = -1; lb = umin * ones(Hc, 1);
umax = 1; ub = umax * ones(Hc, 1);

%% prealloc
rng(1);
Vw_d = zeros(N,1); 
U_d = zeros(N,3);
U0 = zeros(Hc, 3);

%% Main Simulation Loop
for k = 1:N

    % time betwen two samples
    tk1 = tk(k);
    tk2 = tk(k+1);
    Tspan = [tk1 tk2];

    % reference sample at time k
    r_k = ref_state_circle(tk1);

    % controllers call

    % PID
    % [ax, ay, ah, pid_errors] = pid_controller(s0, r_k, Ts, pid_errors);

    % SF
    % [ax, ay, ah, Kxy, Kz] = sf_controller(s0, r_k);

    % MPC
    [ax, ay, ah] = mpc_controller(s0, r_k, A, B, Hp, Hc, Q, R, lb, ub, U0);

    % apply accelarations & control storage
    u_k = [ax ay ah];
    U_d(k,:) = u_k;

    % wind disturbances
    Vw_d(k) = wind_disturbances(s0, params);

    % UAV Dynamics
    ode_fun = @(t,s) uav_dynamics(t, s, u_k, Vw_d(k), params);
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
end

% UAV data log
logs = uav_datalog(t_total, S_total, tk, U_d, Vw_d, @ref_state_circle, params);

% performance metrics
metrics = performance_metrics(t_total, Ts, logs)

% plots
plots_func(t_total, logs, metrics);