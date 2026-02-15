clc; clear; close all;

% UAV parameters
params = uav_params();

% Initial UAV conditions
x0 = 0;
y0 = 0; 
h0 = 60;
Vg0 = 30; 
gamma0 = 0; 
psi0 = 0;
dx0 = Vg0 * cos(gamma0) * cos(psi0);
dy0 = Vg0 * cos(gamma0) * sin(psi0);
dh0 = Vg0 * sin(gamma0);

% initial system state vector
ss0 = [x0; y0; h0; Vg0; gamma0; psi0];

% initial control state vector
cs0 = [x0; y0; h0; dx0; dy0; dh0];

% Simulation Parameteres
T = 30; % simulation time
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
Hc = 10;

% Cost weights for cost function
Q = diag([3 3 2 4 4 3]); % 6x6 weight for tracking_cost
R = diag([5 5 4]);       % 3x3 weight for control_cost
Rd = diag([2 2 1]);     % 3x3 weight for dU_cost             

% Constraints & bounds for optimization (fmincon)
umin_xy = -10; umax_xy = 10;
umin_h = -10; umax_h = 10;
lb = repmat([umin_xy; umin_xy; umin_h], Hc, 1);
ub = repmat([umax_xy; umax_xy; umax_h], Hc, 1);

%% prealloc
rng(1); % seed for randn in wind_disturbances
Vw_d = zeros(N,1); % wind log alloc
U_d = zeros(N,3); % control log alloc
U0 = zeros(3*Hc,1); % first control signal guess for mpc
u_prev = zeros(3,1); % for dU in mpc_cost_func
J_k = zeros(N,1); % mpc_cost_func value log
exitflag_k = zeros(N,1); % exitflag mpc_cost_func log

%% Main Simulation Loop
for k = 1:N

    % time betwen two samples
    tk1 = tk(k);
    tk2 = tk(k+1);
    Tspan = [tk1 tk2];

    % reference sample at time k
    r_k = ref_state_circle(tk1);

    % wind disturbances
    Vw_d(k) = wind_disturbances(ss0, params);

    % PID
    % [ax, ay, ah, pid_errors] = pid_controller(cs0, r_k, Ts, pid_errors);

    % SF
    % [ax, ay, ah, Kxy, Kz] = sf_controller(cs0, r_k);

    % MPC
    t = (k-1)*Ts;
    ref_prev = ref_state_circle(t); % 9x1
    a_ref_prev = ref_prev(7:9); % 3x1
    ref_mpc = mpc_ref_window(t, Hp, Ts, @ref_state_circle);
    [ax, ay, ah, U0, J, exitflag, output] = ...
    mpc_controller(params, cs0, ref_mpc, A, B, Q, R, Rd, Hp, Hc, lb, ub, U0, Vw_d(k), u_prev, a_ref_prev);
    J_k(k) = J;
    exitflag_k(k) = exitflag;

    % apply accelarations & control storage
    u_k = [ax ay ah];
    U_d(k,:) = u_k;
    u_prev = u_k';

    % UAV Dynamics
    ode_fun = @(t,s) uav_dynamics(t, s, u_k, Vw_d(k), params);
    [t_seg, S_seg] = ode45(ode_fun, Tspan, ss0);

    % final system state values become next initial system state vector
    ss0 = S_seg(end,:).';

    % next initial control state vector (ss --> cs)
    dx0 = ss0(4) * cos(ss0(5)) * cos(ss0(6));
    dy0 = ss0(4) * cos(ss0(5)) * sin(ss0(6));
    dh0 = ss0(4) * sin(ss0(5));
    cs0 = [ss0(1); ss0(2); ss0(3); dx0; dy0; dh0];

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
metrics = performance_metrics(t_total, Ts, logs);

% plots
plots_func(t_total, logs, metrics, J_k, exitflag_k);