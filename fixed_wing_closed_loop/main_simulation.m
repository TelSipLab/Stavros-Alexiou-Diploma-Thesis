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
s0 = [x0; y0; h0; Vg0; gamma0; psi0]; % state vector for pid and sf
dx0 = Vg0 * cos(gamma0) * cos(psi0);
dy0 = Vg0 * cos(gamma0) * sin(psi0);
dh0 = Vg0 * sin(gamma0);
s0 = [x0; y0; h0; dx0; dy0; dh0]; % state vector for mpc

% Simulation Parameteres
T = 20; % simulation time
Ts = 0.1; % sampling time
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
Hp = 10; 
Hc = 5;

% Cost weights for cost function
Q = diag([1 1 1 0.2 0.2 0.2]); % 6x6 - weight for tracking cost
R = 10*eye(3); % 3x3 - weight for control cost              

% Constraints & bounds for optimization (fmincon)
umin_xy = -100; umax_xy = 100;
umin_h = -100; umax_h = 100;
lb = repmat([umin_xy; umin_xy; umin_h], Hc, 1);
ub = repmat([umax_xy; umax_xy; umax_h], Hc, 1);

%% prealloc
rng(1);
Vw_d = zeros(N,1); % wind log alloc
U_d = zeros(N,3); % control log alloc
U0 = zeros(3*Hc,1); % first control signal guess for mpc

%% Main Simulation Loop
for k = 1:N

    % time betwen two samples
    tk1 = tk(k);
    tk2 = tk(k+1);
    Tspan = [tk1 tk2];

    % reference sample at time k
    r_k = ref_state_circle(tk1);

    % PID
    % [ax, ay, ah, pid_errors] = pid_controller(s0, r_k, Ts, pid_errors);

    % SF
    % [ax, ay, ah, Kxy, Kz] = sf_controller(s0, r_k);

    % ref window for MPC call
    t = (k-1)*Ts;
    r_mpc = mpc_ref_window(t, Hp, Ts, @ref_state_circle);  % 6 x Hp

    % MPC
    [ax, ay, ah, U0, exitflag, output] = ...
        mpc_controller(s0, r_mpc, A, B, Q, R, Hp, Hc, lb, ub, U0);

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