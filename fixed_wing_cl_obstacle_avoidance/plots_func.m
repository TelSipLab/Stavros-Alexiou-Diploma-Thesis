function plots_func(t, logs, metrics, J, exitflag, obstacles, obst_params)
%% log extraction
% states
x = logs.x; y = logs.y; h = logs.h;
Vg = logs.Vg; gamma = logs.gamma; psi = logs.psi;

% velocities
xdot = logs.xdot;
ydot = logs.ydot;
hdot = logs.hdot;

% reference
rx = logs.ref_cont(:,1); ry = logs.ref_cont(:,2); rh = logs.ref_cont(:,3);

% controller outputs
xddot = logs.U(:,1); yddot = logs.U(:,2); hddot = logs.U(:,3);

% lyapunov function values
Vx = logs.Vx; Vy = logs.Vy; Vh = logs.Vh; Vtot = logs.Vtot;

%% UAV Constraints
C = uav_constraints();

%% fig1: 3D Trajectory
figure;
plot3(x, y, h, 'b-', 'LineWidth', 1.6); hold on;
plot3(rx, ry, rh, 'g--', 'LineWidth', 1.4);
plot3(x(1), y(1), h(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), h(end), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
xlabel('x [m]'); ylabel('y [m]'); zlabel('h [m]');
title('UAV 3D Trajectory vs Reference'); grid on; axis equal;
legend('UAV','Reference','Start','End','Location','best');

%% fig2: State Evolution + state constraints + reference
figure;

% uav position (x,y,h)
subplot(6,1,1); plot(t, x,'b','LineWidth',1.4); grid on; ylabel('x [m]');
title('position x');
hold on; plot(t,rx,'g--','LineWidth',1.4); legend('x','reference');
subplot(6,1,2); plot(t, y,'b','LineWidth',1.4); grid on; ylabel('y [m]');
title('position y');
hold on; plot(t,ry,'g--','LineWidth',1.4); legend('y','reference');
subplot(6,1,3); plot(t,h,'b','LineWidth',1.4); grid on; ylabel('h [m]');
title('position h');
hold on; plot(t,rh,'g--','LineWidth',1.4); legend('h','reference');

% Vg
subplot(6,1,4); plot(t, Vg,'b','LineWidth',1.4); grid on; hold on;
yline(C.Vg_min,'r--','LineWidth',1.4);
yline(C.Vg_max,'r--','LineWidth',1.4);
ylabel('V_g [m/s]'); title('Ground Speed');
legend('V_g','limits'); hold off;

% gamma
subplot(6,1,5); plot(t, rad2deg(gamma),'b','LineWidth',1.4); grid on;
yline(rad2deg(C.gamma_min),'r--','LineWidth',1.4) ;
yline(rad2deg(C.gamma_max),'r--','LineWidth',1.4);
ylabel('\gamma [deg]'); title('Flight Path Angle');
legend('\gamma','limits'); hold off;

% psi
subplot(6,1,6);
psi_deg = mod(rad2deg(psi) + 180, 360) - 180; % psi wrap [-180,180]
plot(t, psi_deg,'b','LineWidth',1.4); grid on;
ylabel('\psi [deg]'); xlabel('Time [s]'); title('Heading Angle');
legend('\psi'); hold off;

sgtitle('UAV State Evolution');

%% fig3: Velocities
figure;
subplot(3,1,1); plot(t, xdot,'LineWidth',1.4); grid on; ylabel('\it\_x [m/s]');
title('Velocities');
subplot(3,1,2); plot(t, ydot,'LineWidth',1.4); grid on; ylabel('\it\_y [m/s]' );
subplot(3,1,3); plot(t, hdot,'LineWidth',1.4); grid on; ylabel('\it\_h [m/s]');
xlabel('Time [s]');

%% fig4: Accelerations (controller outputs)
figure;
subplot(3,1,1); plot(t, xddot, 'LineWidth',1.4); grid on; ylabel('\it\_x [m/s^2]');
title('Accelerations (controller outputs)');
subplot(3,1,2); plot(t, yddot,'LineWidth',1.4); grid on; ylabel('\it\_y [m/s^2]');
subplot(3,1,3); plot(t, hddot, 'LineWidth',1.4); grid on; ylabel('\it\_h [m/s^2]');
xlabel('Time [s]');

%% fig5: Th, ng, phib
figure;
Th = logs.Th_d_zoh;
ng = logs.ng_d_zoh;
phib = logs.phib_d_zoh;

% Th
subplot(3,1,1);
plot(t, Th, 'LineWidth', 1.4); grid on; hold on;
yline(C.Th_min,'r--','LineWidth',1.4);
yline(C.Th_max,'r--','LineWidth',1.4);
ylabel('T_h [N]'); title('Engine Thrust');
legend('T_h','limits','Location','best');
hold off;

% ng
subplot(3,1,2);
plot(t, ng, 'LineWidth', 1.4); grid on; hold on;
yline(C.ng_min,'r--','LineWidth',1.4);
yline(C.ng_max,'r--','LineWidth',1.4);
ylabel('n_g [G]'); title('G-load');
legend('n_g','limits','Location','best');
hold off;

% phib
subplot(3,1,3);
plot(t, rad2deg(phib), 'LineWidth', 1.4); grid on; hold on;
yline(rad2deg(C.phib_min),'r--','LineWidth',1.4);
yline(rad2deg(C.phib_max),'r--','LineWidth',1.4);
ylabel('\phi_b [deg]'); xlabel('Time [s]'); title('Bank Angle');
legend('\phi_b','limits','Location','best');
hold off;

%% fig6: Euclidean Distance Evolution
figure;
plot(t, metrics.ED,'LineWidth',1.4); grid on;
ylabel('Euclidean Distance'); xlabel('Time [s]');
title('Euclidean Distance Evolution');

%% fig7: Objective value J
figure; plot(J); grid on;
title('Objective value J over time'); xlabel('k'); ylabel('J');

%% fig8: exitflag value
figure; stairs(exitflag); grid on;
title('exitflag value over time'); xlabel('k'); ylabel('exitflag');

%% fig9: Lyapunov functions
figure;

subplot(4,1,1);
plot(t, Vx, 'LineWidth', 1.4); grid on;
ylabel('V_x'); title('Lyapunov function for x-axis');

subplot(4,1,2);
plot(t, Vy, 'LineWidth', 1.4); grid on;
ylabel('V_y'); title('Lyapunov function for y-axis');

subplot(4,1,3);
plot(t, Vh, 'LineWidth', 1.4); grid on;
ylabel('V_h'); title('Lyapunov function for h-axis');

subplot(4,1,4);
plot(t, Vtot, 'LineWidth', 1.4); grid on;
ylabel('V_{tot}'); xlabel('Time [s]');
title('Total Lyapunov function');

%% fig10: Obstacle collision clearance
obst_dist = zeros(numel(t), numel(obstacles));
for i = 1:numel(obstacles)
    dx_obst = x - obstacles(i).pos(1);
    dy_obst = y - obstacles(i).pos(2);
    dh_obst = h - obstacles(i).pos(3);
    obst_dist(:,i) = sqrt(dx_obst.^2 + dy_obst.^2 + dh_obst.^2);
end

min_obst_dist = min(obst_dist, [], 2);
collision_bound = obst_params.r_uav + obst_params.r_obst;
obst_clearance = min_obst_dist - collision_bound;

figure;
plot(t, obst_clearance, 'b-', 'LineWidth', 1.6); hold on;
yline(0, 'r--', 'LineWidth', 1.4);
grid on;
xlim([t(1), t(end)]);
ylim([min(-0.5, min(obst_clearance) - 0.2), max(obst_clearance) + 5]);
xlabel('Time [sec]');
ylabel('Distance [m]');
title('Minimum Distance Between the UAV and Any Obstacle');
legend('Distance to Obstacle', 'Collision Bound', 'Location', 'northeast');

zoom_idx = find(obst_clearance <= 1);
near_collision_zoom = ~isempty(zoom_idx);
if isempty(zoom_idx)
    [~, min_clearance_idx] = min(obst_clearance);
    zoom_idx = max(1, min_clearance_idx-20):min(numel(t), min_clearance_idx+20);
end
zoom_time_min = max(t(1), t(zoom_idx(1)) - 0.5);
zoom_time_max = min(t(end), t(zoom_idx(end)) + 0.5);
zoom_mask = (t >= zoom_time_min) & (t <= zoom_time_max);

zoom_axis = axes('Position', [0.32 0.49 0.36 0.27]);
plot(zoom_axis, t, obst_clearance, 'b-', 'LineWidth', 1.4); hold(zoom_axis, 'on');
yline(zoom_axis, 0, 'r--', 'LineWidth', 1.2);
grid(zoom_axis, 'on');
xlim(zoom_axis, [zoom_time_min, zoom_time_max]);
zoom_clearance_min = min(obst_clearance(zoom_mask));
zoom_clearance_max = max(obst_clearance(zoom_mask));
if near_collision_zoom
    ylim(zoom_axis, [min(-0.05, zoom_clearance_min - 0.05), ...
        max(1, min(1.2, zoom_clearance_max + 0.1))]);
else
    zoom_clearance_margin = max(0.1, 0.05 * (zoom_clearance_max - zoom_clearance_min));
    ylim(zoom_axis, [zoom_clearance_min - zoom_clearance_margin, ...
        zoom_clearance_max + zoom_clearance_margin]);
end
hold off;

end
