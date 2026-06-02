clc; clear; close all;

% uav parameters
params = uav_params();

% obstacle avoidance setup
[obstacles, obst_params] = obstacle_params(@ref_state_hippodrome);

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

% Cost weights for cost function (tunned withoute obstacle avoidance)
% Q  = diag([1 1 1 4 4 4]);  % 6x6 weight for tracking_cost
% R  = diag([4.2 4.2 22]);   % 3x3 weight for control_cost
% Rd = diag([3 3 1.2]);      % 3x3 weight for dU_cost 

Q  = 0.1*diag([1 1 1 10 10 10]);  % 6x6 weight for tracking_cost
R  = 10*diag([1 2 1.5]);      % 3x3 weight for control_cost
Rd = diag([1 2 1.5]);     % 3x3 weight for dU_cost  

% constraints & bounds for the optimization problem (fmincon)
lb = repmat([-5; -5; -5], Hc, 1); % lower bounds
ub = repmat([5; 5; 5], Hc, 1);    % upper bounds
a_con = [lb(1:3) ub(1:3)];

% da constraints: rows [dax; day; dah], columns [min max]
dax_min = -1; dax_max = 1;
day_min = -1; day_max = 1;
dah_min = -1; dah_max = 1;
da_con = [dax_min dax_max; day_min day_max; dah_min dah_max];

% contractive mpc
alpha = 1;

%% prealloc vectors before the main simulation loop
rng(1);                     % seed for randn in wind_disturbances
Vw_d = zeros(N,1);          % wind log alloc
U_d = zeros(N,3);           % control log alloc
eucl_dist_prev = 0;         % euclidian distance evaluation for printf
U0 = zeros(3*Hc,1);         % mpc first control signal guess
u_prev = zeros(3,1);        % mpc cost function dU 
J = zeros(N,1);             % mpc cost function value log
J_tr = zeros(N,1);          % mpc tracking cost log
J_con = zeros(N,1);         % mpc control cost log
J_du = zeros(N,1);          % mpc dU cost log
J_apf = zeros(N,1);         % mpc APF cost log
exitflag = zeros(N,1);      % mpc exitflag log
C_VBF = zeros(N,1);         % mpc VBF contractive constraint check
C_Th_check = zeros(N,1);    % thrust constraint check
C_ng_check = zeros(N,1);    % g-load constraint check
C_phib_check = zeros(N,1);  % bank angle constraint check
C_Vg_check = zeros(N,1);    % ground speed constraint check
C_gamma_check = zeros(N,1); % flight path angle constraint check
C_da_check = zeros(N,1);    % delta acceleration constraint check
C_obst_check = zeros(N,1);  % obstacle constraint check
obst_dist_d = zeros(N, numel(obstacles)); % discrete obstacle distance log

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
    da_con, alpha, U0, Vw_d(k), u_prev, a_ref_prev, obstacles, obst_params);

    [~, cost_terms] = mpc_cost_func(U_opt, u_prev, cs0, A, B, Q, R, Rd, ...
        Hp, Hc, ref_mpc, a_ref_prev, obstacles, obst_params);
    J_tr(k) = cost_terms.tr_cost;
    J_con(k) = cost_terms.con_cost;
    J_du(k) = cost_terms.du_cost;
    J_apf(k) = cost_terms.apf_cost;

    CS = mpc_state_prediction(U_opt, cs0, A, B, Hp, Hc);
    C_VBF(k) = mpc_contractive_constraint(cs0, ref_mpc, CS, alpha, obstacles, obst_params);
    [~, ~, C_groups] = mpc_constraints(U_opt, u_prev, cs0, ref_mpc, A, B, Hp, Hc, ...
        da_con, alpha, Vw_d(k), params, obstacles, obst_params);
    C_Th_check(k) = C_groups.Th;
    C_ng_check(k) = C_groups.ng;
    C_phib_check(k) = C_groups.phib;
    C_Vg_check(k) = C_groups.Vg;
    C_gamma_check(k) = C_groups.gamma;
    C_da_check(k) = C_groups.da;
    C_obst_check(k) = C_groups.obst;

    for i = 1:numel(obstacles)
        obst_dist_d(k,i) = norm(cs0(1:3) - obstacles(i).pos);
    end
    [nearest_obst_dist, nearest_obst_idx] = min(obst_dist_d(k,:));

    eucl_dist = norm(cs0(1:3)-r_k(1:3));
    eucl_dist_change = eucl_dist - eucl_dist_prev;
    eucl_dist_prev = eucl_dist;
    fprintf(['k = %4d/%4d | MPC | J = %8.1f | J_Q = %8.1f | J_R = %8.1f | ', ...
        'J_Rd = %8.1f | J_APF = %8.1f | exit = %2d | ed = %6.1f | ', ...
        'ded = %6.1f | obst = %d | obst_d = %6.1f\n'], ...
        k, N, J(k), J_tr(k), J_con(k), J_du(k), J_apf(k), exitflag(k), ...
        eucl_dist, eucl_dist_change, nearest_obst_idx, nearest_obst_dist);
   
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

%% log and metrics

% UAV data log
logs = uav_datalog(t_total, S_total, tk, U_d, Vw_d, @ref_state_hippodrome, params);

% performance metrics
metrics = performance_metrics(t_total, Ts, logs)

fprintf('\nMPC constraint summary (max C <= 0 means satisfied):\n');
fprintf('Th      = %12.6f\n', max(C_Th_check));
fprintf('ng      = %12.6f\n', max(C_ng_check));
fprintf('phib    = %12.6f\n', max(C_phib_check));
fprintf('Vg      = %12.6f\n', max(C_Vg_check));
fprintf('gamma   = %12.6f\n', max(C_gamma_check));
fprintf('da      = %12.6f\n', max(C_da_check));
fprintf('obst    = %12.6f\n', max(C_obst_check));
fprintf('VBF     = %12.6f\n', max(C_VBF));

constraint_tol = 1e-6;
fprintf('\nMPC constraint violation intervals (C > %.1e):\n', constraint_tol);
print_constraint_diagnostics('Th', C_Th_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('ng', C_ng_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('phib', C_phib_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('Vg', C_Vg_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('gamma', C_gamma_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('da', C_da_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('obst', C_obst_check, tk(1:end-1), constraint_tol);
print_constraint_diagnostics('VBF', C_VBF, tk(1:end-1), constraint_tol);

%% obstacle avoidance evaluation

% obstacle distance calculation
obst_dist = zeros(length(t_total), numel(obstacles));
for i = 1:numel(obstacles)
    delta_x_obst = logs.x - obstacles(i).pos(1);
    delta_y_obst = logs.y - obstacles(i).pos(2);
    delta_h_obst = logs.h - obstacles(i).pos(3);
    obst_dist(:,i) = sqrt(delta_x_obst.^2 + delta_y_obst.^2 + delta_h_obst.^2);
end

% minimum distance and collision clearance calculation
[min_obst_dist, min_obst_idx] = min(obst_dist, [], 1);
min_obst_time = t_total(min_obst_idx);
collision_bound = obst_params.r_uav + obst_params.r_obst;
obst_clearance = min_obst_dist - collision_bound;

% obstacle avoidance summary
fprintf('\nObstacle avoidance summary:\n');
for i = 1:numel(obstacles)
    if isfield(obstacles, 'id')
        obstacle_id = obstacles(i).id;
    else
        obstacle_id = i;
    end
    fprintf('Obstacle %d | t_obst = %8.3f s | t_min = %8.3f s | min distance = %10.3f m | clearance = %10.3f m\n', ...
        obstacle_id, obstacles(i).t, min_obst_time(i), min_obst_dist(i), obst_clearance(i));
end

%% plots
plots_func(t_total, logs, metrics, J, exitflag, obstacles, obst_params, a_con, da_con);
