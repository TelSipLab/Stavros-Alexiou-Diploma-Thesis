clear all
close all
clc

% Select these parameters to define the entire trajectory
v_cruise = 32;   % Cruising speed [m/s]
T_straight = 60; % Duration of the straight sections [s]
R_turn = 500;    % Turn radius in meters (increase this to decrease the maximum required acceleration)
dt = 0.05;       % Time step
t_total = 500;   % Total time of trajectory

g = 9.81; % Gravity [m/s^2]

T_turn = (pi * R_turn) / v_cruise; % Time required for 180-degree turn
t_vec = 0:dt:t_total;

N = length(t_vec);
pos = zeros(2, N); vel = zeros(2, N); acc = zeros(2, N);

for i = 1:N
    % Get trajectory profile
    [pos(:,i), vel(:,i), acc(:,i)] = get_state(t_vec(i), v_cruise, T_straight, R_turn, T_turn);
end

% Calculate Ground Speed
Vg = sqrt(vel(1,:).^2 + vel(2,:).^2);

% Calculate Heading Rate (dchi/dt)
%(vx*ay - vy*ax) / (vx^2 + vy^2)
dchi = (vel(1,:).*acc(2,:) - vel(2,:).*acc(1,:)) ./ (Vg.^2);

% Calculate Banking Angle in Degrees
phi_rad = atan2((Vg .* dchi), g);
phi_deg = rad2deg(phi_rad);

% Plot Banking Angle
figure('Color', 'w');
subplot(3,1,1)
plot(t_vec, phi_deg, 'LineWidth', 2, 'Color', [0.85, 0.33, 0.1]);
grid on;
title('Required Banking Angle');
xlabel('Time [s]'); ylabel('Bank Angle [deg]');
subplot(3,1,2)
plot(t_vec, Vg, 'LineWidth', 2, 'Color', [0.85, 0.33, 0.1]);
grid on;
title('Required Ground Speed');
xlabel('Time [s]'); ylabel('Ground Speed [m/sec]');
subplot(3,1,3)
plot(t_vec, rad2deg(dchi), 'LineWidth', 2, 'Color', [0.85, 0.33, 0.1]);
grid on;
title('Required Heading Rate');
xlabel('Time [s]'); ylabel('Heading Rate [deg/sec]');

figure('Color', 'w', 'Position', [100, 100, 1100, 800]);

subplot(2,2,1); plot(pos(1,:), pos(2,:), 'b', 'LineWidth', 2); grid on;
title(sprintf('XY Path (v=%d, R=%d)', v_cruise, R_turn));
xlabel('X [m]'); ylabel('Y [m]'); ylim([min(pos(2,:))-50 max(pos(2,:)+50)])
xlim([min(pos(1,:))-50 max(pos(1,:)+50)])

subplot(2,2,2); plot(t_vec, pos, 'LineWidth', 1.5); grid on;
title('Position vs Time'); legend('x(t)', 'y(t)');

subplot(2,2,3); plot(t_vec, vel, 'LineWidth', 1.5); grid on;
title('Velocity (C^1 Continuous)'); ylabel('m/s'); legend('v_x', 'v_y');

subplot(2,2,4); plot(t_vec, acc, 'LineWidth', 1.5); grid on;
title('Acceleration (C^2 Continuous)'); ylabel('m/s^2');
hold on;

%% --- TRAJECTORY FUNCTION ---
function [p, v, a] = get_state(t, v0, Ts, R, Tt)
    % Times that transitions occur
    t1 = Ts;
    t2 = t1 + Tt;
    t3 = t2 + Ts;
    t4 = t3 + Tt;
    
    t_mod = mod(t, t4);

    if t_mod <= t1
        % Segment 1: Bottom Straight
        theta = 0; dtheta = 0; ddtheta = 0;
        p = [v0 * t_mod; 0];
    elseif t_mod <= t2
        % Segment 2: First Turn
        tr = t_mod - t1;
        [s, ds, dds] = quintic_scaling(tr, Tt);
        theta = pi * s;
        dtheta = pi * ds;
        ddtheta = pi * dds;
        p = [v0*Ts + v0*integral(@(tau) cos(pi*quintic_s(tau, Tt)), 0, tr);
               0   + v0*integral(@(tau) sin(pi*quintic_s(tau, Tt)), 0, tr)];
    elseif t_mod <= t3
        % Segment 3: Top Straight
        tr = t_mod - t2;
        theta = pi; dtheta = 0; ddtheta = 0;
        xe = v0*Ts + v0*integral(@(tau) cos(pi*quintic_s(tau, Tt)), 0, Tt);
        ye = v0*integral(@(tau) sin(pi*quintic_s(tau, Tt)), 0, Tt);
        p = [xe - v0 * tr; ye];
    else
        % Segment 4: Second Turn
        tr = t_mod - t3;
        [s, ds, dds] = quintic_scaling(tr, Tt);
        theta = pi + pi * s;
        dtheta = pi * ds;
        ddtheta = pi * dds;
        ye = v0*integral(@(tau) sin(pi*quintic_s(tau, Tt)), 0, Tt);
        p = [0  + v0*integral(@(tau) cos(pi + pi*quintic_s(tau, Tt)), 0, tr);
             ye + v0*integral(@(tau) sin(pi + pi*quintic_s(tau, Tt)), 0, tr)];
    end

    v = [v0 * cos(theta); v0 * sin(theta)];
    a = [-v0 * sin(theta) * dtheta; v0 * cos(theta) * dtheta];
end

function [s, ds, dds] = quintic_scaling(t, T)
    tau = t/T;
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    ds = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
    dds = (60*tau - 180*tau.^2 + 120*tau.^3)/T^2;
end

function s = quintic_s(t, T)
    tau = t/T;
    s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
end