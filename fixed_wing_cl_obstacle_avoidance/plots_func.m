function plots_func(t, logs, metrics, J, exitflag, obstacles, obst_params, a_con, da_con)
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
rdx = logs.ref_vel_cont(:,1); rdy = logs.ref_vel_cont(:,2); rdh = logs.ref_vel_cont(:,3);
rd2x = logs.ref_acc_cont(:,1); rd2y = logs.ref_acc_cont(:,2); rd2h = logs.ref_acc_cont(:,3);
rVg = sqrt(rdx.^2 + rdy.^2 + rdh.^2);
rGamma = asin(rdh ./ rVg);
rPsi = atan2(rdy, rdx);

% controller outputs
xddot = logs.U(:,1); yddot = logs.U(:,2); hddot = logs.U(:,3);

% lyapunov function values
Vx = logs.Vx; Vy = logs.Vy; Vh = logs.Vh; Vtot = logs.Vtot;

%% UAV Constraints
C = uav_constraints();

if nargin < 8 || isempty(a_con)
    a_con = [-Inf Inf; -Inf Inf; -Inf Inf];
end
if nargin < 9 || isempty(da_con)
    da_con = [-Inf Inf; -Inf Inf; -Inf Inf];
end

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
plot(t, rVg,'g--','LineWidth',1.4);
yline(C.Vg_min,'r--','LineWidth',1.4);
yline(C.Vg_max,'r--','LineWidth',1.4);
ylabel('V_g [m/s]'); title('Ground Speed');
legend('V_g','reference','limits'); hold off;

% gamma
subplot(6,1,5); plot(t, rad2deg(gamma),'b','LineWidth',1.4); grid on; hold on;
plot(t, rad2deg(rGamma),'g--','LineWidth',1.4);
yline(rad2deg(C.gamma_min),'r--','LineWidth',1.4) ;
yline(rad2deg(C.gamma_max),'r--','LineWidth',1.4);
ylabel('\gamma [deg]'); title('Flight Path Angle');
legend('\gamma','reference','limits'); hold off;

% psi
subplot(6,1,6);
psi_deg = mod(rad2deg(psi) + 180, 360) - 180; % psi wrap [-180,180]
rPsi_deg = mod(rad2deg(rPsi) + 180, 360) - 180; % reference psi wrap [-180,180]
plot(t, psi_deg,'b','LineWidth',1.4); grid on; hold on;
plot(t, rPsi_deg,'g--','LineWidth',1.4);
ylabel('\psi [deg]'); xlabel('Time [s]'); title('Heading Angle');
legend('\psi','reference'); hold off;

sgtitle('UAV State Evolution');

%% fig3: Velocities
figure;
subplot(3,1,1); plot(t, xdot,'LineWidth',1.4); grid on; ylabel('\it\_x [m/s]');
title('Velocities');
hold on; plot(t, rdx,'g--','LineWidth',1.4); legend('\it\_x','reference');
subplot(3,1,2); plot(t, ydot,'LineWidth',1.4); grid on; ylabel('\it\_y [m/s]' );
hold on; plot(t, rdy,'g--','LineWidth',1.4); legend('\it\_y','reference');
subplot(3,1,3); plot(t, hdot,'LineWidth',1.4); grid on; ylabel('\it\_h [m/s]');
hold on; plot(t, rdh,'g--','LineWidth',1.4); legend('\it\_h','reference');
xlabel('Time [s]');

%% fig4: Accelerations (controller outputs)
figure;
subplot(3,1,1); plot(t, xddot, 'LineWidth',1.4); grid on; ylabel('\it\_x [m/s^2]');
title('Accelerations (controller outputs)');
hold on; plot(t, rd2x,'g--','LineWidth',1.4);
yline(a_con(1,1),'r--','LineWidth',1.4);
yline(a_con(1,2),'r--','LineWidth',1.4);
legend('\it\_x','reference','limits');
subplot(3,1,2); plot(t, yddot,'LineWidth',1.4); grid on; ylabel('\it\_y [m/s^2]');
hold on; plot(t, rd2y,'g--','LineWidth',1.4);
yline(a_con(2,1),'r--','LineWidth',1.4);
yline(a_con(2,2),'r--','LineWidth',1.4);
legend('\it\_y','reference','limits');
subplot(3,1,3); plot(t, hddot, 'LineWidth',1.4); grid on; ylabel('\it\_h [m/s^2]');
hold on; plot(t, rd2h,'g--','LineWidth',1.4);
yline(a_con(3,1),'r--','LineWidth',1.4);
yline(a_con(3,2),'r--','LineWidth',1.4);
legend('\it\_h','reference','limits');
xlabel('Time [s]');

%% fig5: Delta accelerations (controller output changes)
tk_u = logs.tk(1:end-1);
U_d = logs.U_d;
ref_acc_d = logs.ref_acc_samp;
dU_d = zeros(size(U_d));
dU_ref = zeros(size(ref_acc_d));
dU_d(1,:) = U_d(1,:);
dU_d(2:end,:) = U_d(2:end,:) - U_d(1:end-1,:);
dU_ref(1,:) = ref_acc_d(1,:);
dU_ref(2:end,:) = ref_acc_d(2:end,:) - ref_acc_d(1:end-1,:);

figure;
subplot(3,1,1); plot(tk_u, dU_d(:,1), 'LineWidth',1.4); grid on; ylabel('\Delta\it\_x [m/s^2]');
title('Delta Accelerations (controller output changes)');
hold on; plot(tk_u, dU_ref(:,1),'g--','LineWidth',1.4);
yline(da_con(1,1),'r--','LineWidth',1.4);
yline(da_con(1,2),'r--','LineWidth',1.4);
legend('\Delta\it\_x','reference','limits');
subplot(3,1,2); plot(tk_u, dU_d(:,2), 'LineWidth',1.4); grid on; ylabel('\Delta\it\_y [m/s^2]');
hold on; plot(tk_u, dU_ref(:,2),'g--','LineWidth',1.4);
yline(da_con(2,1),'r--','LineWidth',1.4);
yline(da_con(2,2),'r--','LineWidth',1.4);
legend('\Delta\it\_y','reference','limits');
subplot(3,1,3); plot(tk_u, dU_d(:,3), 'LineWidth',1.4); grid on; ylabel('\Delta\it\_h [m/s^2]');
hold on; plot(tk_u, dU_ref(:,3),'g--','LineWidth',1.4);
yline(da_con(3,1),'r--','LineWidth',1.4);
yline(da_con(3,2),'r--','LineWidth',1.4);
legend('\Delta\it\_h','reference','limits');
xlabel('Time [s]');

%% fig6: Th, ng, phib
figure;
Th = logs.Th_d_zoh;
ng = logs.ng_d_zoh;
phib = logs.phib_d_zoh;

% Th
subplot(3,1,1);
plot(t, Th, 'LineWidth', 1.4); grid on; hold on;
plot(t, logs.ref_Th_cont, 'g--', 'LineWidth', 1.4);
yline(C.Th_min,'r--','LineWidth',1.4);
yline(C.Th_max,'r--','LineWidth',1.4);
ylabel('T_h [N]'); title('Engine Thrust');
legend('T_h','reference','limits','Location','best');
hold off;

% ng
subplot(3,1,2);
plot(t, ng, 'LineWidth', 1.4); grid on; hold on;
plot(t, logs.ref_ng_cont, 'g--', 'LineWidth', 1.4);
yline(C.ng_min,'r--','LineWidth',1.4);
yline(C.ng_max,'r--','LineWidth',1.4);
ylabel('n_g [G]'); title('G-load');
legend('n_g','reference','limits','Location','best');
hold off;

% phib
subplot(3,1,3);
plot(t, rad2deg(phib), 'LineWidth', 1.4); grid on; hold on;
plot(t, rad2deg(logs.ref_phib_cont), 'g--', 'LineWidth', 1.4);
yline(rad2deg(C.phib_min),'r--','LineWidth',1.4);
yline(rad2deg(C.phib_max),'r--','LineWidth',1.4);
ylabel('\phi_b [deg]'); xlabel('Time [s]'); title('Bank Angle');
legend('\phi_b','reference','limits','Location','best');
hold off;

%% fig7: Euclidean Distance Evolution
figure;
plot(t, metrics.ED,'LineWidth',1.4); grid on;
ylabel('Euclidean Distance'); xlabel('Time [s]');
title('Euclidean Distance Evolution');

%% fig8: Objective value J
figure; plot(J); grid on;
title('Objective value J over time'); xlabel('k'); ylabel('J');

%% fig9: exitflag value
figure; stairs(exitflag); grid on;
title('exitflag value over time'); xlabel('k'); ylabel('exitflag');

%% fig10: Lyapunov functions
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

%% fig11: Obstacle collision clearance
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
clearance_layout = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

main_axis = nexttile(clearance_layout, [1 2]);
plot(main_axis, t, obst_clearance, 'b-', 'LineWidth', 1.6); hold(main_axis, 'on');
yline(main_axis, 0, 'r--', 'LineWidth', 1.4);
grid(main_axis, 'on');
xlim(main_axis, [t(1), t(end)]);
ylim(main_axis, [min(-0.5, min(obst_clearance) - 0.2), max(obst_clearance) + 5]);
xlabel(main_axis, 'Time [sec]');
ylabel(main_axis, 'Distance [m]');
title(main_axis, 'Minimum Distance Between the UAV and Any Obstacle');
legend(main_axis, 'Distance to Obstacle', 'Collision Bound', 'Location', 'northeast');
hold(main_axis, 'off');

num_zoom_obstacles = min(4, numel(obstacles));
for i = 1:num_zoom_obstacles
    obstacle_clearance = obst_dist(:,i) - collision_bound;
    zoom_idx = find(obstacle_clearance <= 1);
    near_collision_zoom = ~isempty(zoom_idx);
    if isempty(zoom_idx)
        [~, min_clearance_idx] = min(obstacle_clearance);
        zoom_idx = max(1, min_clearance_idx-20):min(numel(t), min_clearance_idx+20);
    end

    zoom_time_min = max(t(1), t(zoom_idx(1)) - 0.5);
    zoom_time_max = min(t(end), t(zoom_idx(end)) + 0.5);
    zoom_mask = (t >= zoom_time_min) & (t <= zoom_time_max);

    zoom_axis = nexttile(clearance_layout);
    plot(zoom_axis, t, obstacle_clearance, 'b-', 'LineWidth', 1.4); hold(zoom_axis, 'on');
    yline(zoom_axis, 0, 'r--', 'LineWidth', 1.2);
    grid(zoom_axis, 'on');
    xlim(zoom_axis, [zoom_time_min, zoom_time_max]);
    zoom_clearance_min = min(obstacle_clearance(zoom_mask));
    zoom_clearance_max = max(obstacle_clearance(zoom_mask));
    if near_collision_zoom
        ylim(zoom_axis, [min(-0.05, zoom_clearance_min - 0.05), ...
            max(1, min(1.2, zoom_clearance_max + 0.1))]);
    else
        zoom_clearance_margin = max(0.1, 0.05 * (zoom_clearance_max - zoom_clearance_min));
        ylim(zoom_axis, [min(0, zoom_clearance_min - zoom_clearance_margin), ...
            zoom_clearance_max + zoom_clearance_margin]);
    end
    xlabel(zoom_axis, 'Time [sec]');
    ylabel(zoom_axis, 'Clearance [m]');
    if isfield(obstacles, 'id')
        obstacle_id = obstacles(i).id;
    else
        obstacle_id = i;
    end
    title(zoom_axis, sprintf('Obstacle %d', obstacle_id));
    hold(zoom_axis, 'off');
end

end
