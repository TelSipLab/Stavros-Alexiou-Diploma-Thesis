clc; clear; close all;

% uav parameters
params = uav_params();

% initial uav conditions
x0 = -20; y0 = -20; h0 = 70;
Vg0 = 27; gamma0 = deg2rad(0); psi0 = deg2rad(0);

% initial system state vector
ss0 = [x0; y0; h0; Vg0; gamma0; psi0];

% initial control state vector
dx0 = Vg0*cos(gamma0)*cos(psi0);
dy0 = Vg0*cos(gamma0)*sin(psi0);
dh0 = Vg0*sin(gamma0);
cs0 = [x0; y0; h0; dx0; dy0; dh0];

% simulation parameteres
T = 220;     % simulation time
Ts = 0.1;  % sampling time
N = T/Ts;    % total samples
tk = 0:Ts:T; % discrete time log

%% pid setup

% previus e errors
s_ref = ref_state_hippodrome(0);
pid_errors.ex_prev = s_ref(1) - x0; 
pid_errors.ey_prev = s_ref(2) - y0; 
pid_errors.eh_prev = s_ref(3) - h0;
% due to: ed[k] = (e[k]-e[k-1])/Ts)

% previus ei errors
pid_errors.ex_int = 0; pid_errors.ey_int = 0; pid_errors.eh_int = 0;

% pid gains
Kp_x = 0.1; Ki_x = 0; Kd_x = 0.7;
Kp_y = 0.1; Ki_y = 0; Kd_y = 0.7;
Kp_h = 0.1; Ki_h = 0; Kd_h = 0.7;
pid_gains = [Kp_x; Kp_y; Kp_h; Ki_x; Ki_y; Ki_h; Kd_x; Kd_y; Kd_h];

%% sf setup
% pole placement using physical parameters / +-2% criterion

Tset_x = 13; zeta_x = 0.95; wn_x = 4.04/(zeta_x*Tset_x);
Tset_y = 13; zeta_y = 0.95; wn_y = 4.04/(zeta_y*Tset_y);
Tset_h = 13; zeta_h = 0.95; wn_h = 4.04/(zeta_h*Tset_h);

k1_x = wn_x^2; k2_x = 2*zeta_x*wn_y;
k1_y = wn_x^2; k2_y = 2*zeta_y*wn_y;
k1_h = wn_x^2; k2_h = 2*zeta_h*wn_x;
Kx = [k1_x k2_x]; Ky = [k1_y k2_y]; Kh = [k1_h k2_h];

%% mpc setup

% discrete-time ed model (forward euler approximation)
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
Hp = 20; Hc = Hp; % trajectory tracking problem

% Cost weights for cost function
Q  = diag([1 1 1 4 4 4]);  % 6x6 weight for tracking_cost
R  = diag([4.2 4.2 22]);   % 3x3 weight for control_cost
Rd = diag([3 3 1.2]);      % 3x3 weight for dU_cost             

% constraints & bounds for the optimization problem (fmincon)
lb = repmat([-5; -5; -5], Hc, 1);  % lower bounds
ub = repmat([5; 5; 5], Hc, 1);     % upper bounds

% da constraints
da_min = -1; da_max = 1; da_con = [da_min; da_max];

% contractive mpc
alpha = 0.8;

%% prealloc vectors before the main simulation loop
rng(1);                 % seed for randn in wind_disturbances
Vw_d = zeros(N,1);      % wind log alloc
U_d = zeros(N,3);       % control log alloc
eucl_dist_prev = 0;     % euclidian distance evaluation for printf
U0 = zeros(3*Hc,1);     % mpc first control signal guess
u_prev = zeros(3,1);    % mpc cost function dU 
J = zeros(N,1);         % mpc cost function value log
exitflag = zeros(N,1);  % mpc exitflag log
C_V = zeros(N,1);       % mpc contractive constraint check

%% main simulation loop
for k = 1:N

    % time betwen two samples
    tk1 = tk(k);
    tk2 = tk(k+1);
    Tspan = [tk1 tk2];

    % reference sample at time k
    r_k = ref_state_hippodrome(tk1);

    % wind disturbances
    Vw_d(k) = wind_disturbances(ss0, params);

    % % PID
    % [ax, ay, ah, pid_errors] = pid_controller(cs0, r_k, Ts, pid_gains, pid_errors);
    % eucl_dist = norm(cs0(1:3)-r_k(1:3));
    % eucl_dist_change = eucl_dist - eucl_dist_prev;
    % eucl_dist_prev = eucl_dist;
    % fprintf('k = %4d/%4d | PID | ed = %12.6f | ed_change = %14.6f\n', ...
    %     k, N, eucl_dist, eucl_dist_change);

    % % SF
    % [ax, ay, ah] = sf_controller(cs0, r_k, Kx, Ky, Kh);
    % eucl_dist = norm(cs0(1:3)-r_k(1:3));
    % eucl_dist_change = eucl_dist - eucl_dist_prev;
    % eucl_dist_prev = eucl_dist;
    % fprintf('k = %4d/%4d | SF | ed = %12.6f | ed_change = %14.6f\n', ...
    %     k, N, eucl_dist, eucl_dist_change);

    % % MPC
    t = (k-1)*Ts;
    ref_prev = ref_state_hippodrome(t);
    a_ref_prev = ref_prev(7:9);
    ref_mpc = mpc_ref_window(t, Hp, Ts, @ref_state_hippodrome);

    [ax, ay, ah, U0, U_opt, J(k), exitflag(k), output] = ...
    mpc_controller(params, cs0, ref_mpc, A, B, Q, R, Rd, Hp, Hc, lb, ub, ...
    da_con, alpha, U0, Vw_d(k), u_prev, a_ref_prev);

    CS = mpc_state_prediction(U_opt, cs0, A, B, Hp, Hc);
    C_V(k) = mpc_contractive_constraint(cs0, ref_mpc, CS, alpha);

    eucl_dist = norm(cs0(1:3)-r_k(1:3));
    eucl_dist_change = eucl_dist - eucl_dist_prev;
    eucl_dist_prev = eucl_dist;
    fprintf('k = %4d/%4d | MPC | J = %14.6f | exit = %2d | ed = %12.6f | ed_change = %14.6f\n', ...
        k, N, J(k), exitflag(k), eucl_dist, eucl_dist_change);
   
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

%% final calculations

% UAV data log
logs = uav_datalog(t_total, S_total, tk, U_d, Vw_d, @ref_state_hippodrome, params);

% performance metrics
metrics = performance_metrics(t_total, Ts, logs)
contractive_con_check = max(C_V)

%% plots
plots_func(t_total, logs, metrics, J, exitflag);