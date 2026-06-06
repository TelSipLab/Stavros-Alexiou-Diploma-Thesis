clc; clear; close all;

%% Saved obstacle-avoidance run
workspace_file = fullfile(fileparts(mfilename('fullpath')), ...
    'run_alpha095_T270_settling.mat');
export_figures = false;

run_data = load(workspace_file);
t = run_data.t_total;
logs = run_data.logs;
metrics = run_data.metrics;
J = run_data.J;
exitflag = run_data.exitflag;
obstacles = run_data.obstacles;
obst_params = run_data.obst_params;
a_con = run_data.a_con;
da_con = run_data.da_con;
C = uav_constraints();

%% Paper style setup
figure_font_name = 'Times New Roman';
axis_font_size = 10;
legend_font_size = 9;
bottom_legend_position = [0.34 0.035 0.32 0.05];

actual_line_width = 1.6;
reference_line_width = 1.8;
limit_line_width = 1.5;
axes_line_width = 1.1;

actual_color = [0.00 0.20 0.90];
reference_color = [0.10 0.80 0.10];
limit_color = [1.00 0.20 0.20];
start_color = [0.00 0.80 0.00];
end_color = [1.00 0.00 0.00];
grid_color = [0.85 0.85 0.85];

output_directory = fullfile(fileparts(mfilename('fullpath')), ...
    'obstacle_avoidance_figures_thesis');
if export_figures && ~exist(output_directory, 'dir')
    mkdir(output_directory);
end
if export_figures
    old_svg_files = dir(fullfile(output_directory, '*.svg'));
    for i = 1:numel(old_svg_files)
        delete(fullfile(output_directory, old_svg_files(i).name));
    end
end

%% Logged signals
x = logs.x; y = logs.y; h = logs.h;
Vg = logs.Vg; gamma = logs.gamma; psi = logs.psi;

xdot = logs.xdot;
ydot = logs.ydot;
hdot = logs.hdot;

rx = logs.ref_cont(:,1); ry = logs.ref_cont(:,2); rh = logs.ref_cont(:,3);
rdx = logs.ref_vel_cont(:,1); rdy = logs.ref_vel_cont(:,2); rdh = logs.ref_vel_cont(:,3);
rd2x = logs.ref_acc_cont(:,1); rd2y = logs.ref_acc_cont(:,2); rd2h = logs.ref_acc_cont(:,3);

rVg = sqrt(rdx.^2 + rdy.^2 + rdh.^2);
rGamma = asin(rdh ./ rVg);
rPsi = atan2(rdy, rdx);
psi_deg = mod(rad2deg(psi) + 180, 360) - 180;
rPsi_deg = mod(rad2deg(rPsi) + 180, 360) - 180;

xddot = logs.U(:,1);
yddot = logs.U(:,2);
hddot = logs.U(:,3);

tk_u = logs.tk(1:end-1).';
U_d = logs.U_d;
ref_acc_d = logs.ref_acc_samp;
dU_d = zeros(size(U_d));
dU_ref = zeros(size(ref_acc_d));
dU_d(1,:) = U_d(1,:);
dU_d(2:end,:) = U_d(2:end,:) - U_d(1:end-1,:);
dU_ref(1,:) = ref_acc_d(1,:);
dU_ref(2:end,:) = ref_acc_d(2:end,:) - ref_acc_d(1:end-1,:);

Th = logs.Th_d_zoh;
ng = logs.ng_d_zoh;
phib_deg = rad2deg(logs.phib_d_zoh);
ref_Th = logs.ref_Th_cont;
ref_ng = logs.ref_ng_cont;
ref_phib_deg = rad2deg(logs.ref_phib_cont);

time_plot_end = t(end);

%% Figure 1: obstacle-avoidance trajectory views
trajectory_figure = figure('Color', 'w');
set(trajectory_figure, 'Units', 'centimeters');
trajectory_figure.Position(3:4) = [14 18];
movegui(trajectory_figure, 'center');
set(trajectory_figure, 'PaperPositionMode', 'auto');

trajectory_xy_axis = subplot(2,1,1, 'Parent', trajectory_figure);
actual_xy_handle = plot(trajectory_xy_axis, x, y, '-', 'Color', actual_color, ...
    'LineWidth', actual_line_width);
hold(trajectory_xy_axis, 'on');
reference_xy_handle = plot(trajectory_xy_axis, rx, ry, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
start_handle = plot(trajectory_xy_axis, x(1), y(1), 'o', 'Color', start_color, ...
    'MarkerFaceColor', start_color, 'MarkerSize', 6);
end_handle = plot(trajectory_xy_axis, x(end), y(end), 'o', 'Color', end_color, ...
    'MarkerFaceColor', end_color, 'MarkerSize', 6);
axis(trajectory_xy_axis, 'tight');
axis(trajectory_xy_axis, 'equal');
pad_xy_axis(trajectory_xy_axis, 0.08);
xlabel(trajectory_xy_axis, 'x [m]');
ylabel(trajectory_xy_axis, 'y [m]');
style_axis(trajectory_xy_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
set(trajectory_xy_axis, 'Position', [0.13 0.72 0.80 0.17]);
add_subplot_label_at(trajectory_xy_axis, '(a)', figure_font_name, axis_font_size, 0.52, -0.52);

trajectory_3d_axis = subplot(2,1,2, 'Parent', trajectory_figure);
plot3(trajectory_3d_axis, x, y, h, '-', 'Color', actual_color, ...
    'LineWidth', actual_line_width);
hold(trajectory_3d_axis, 'on');
plot3(trajectory_3d_axis, rx, ry, rh, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
plot3(trajectory_3d_axis, x(1), y(1), h(1), 'o', 'Color', start_color, ...
    'MarkerFaceColor', start_color, 'MarkerSize', 6);
plot3(trajectory_3d_axis, x(end), y(end), h(end), 'o', 'Color', end_color, ...
    'MarkerFaceColor', end_color, 'MarkerSize', 6);
axis(trajectory_3d_axis, 'tight');
view(trajectory_3d_axis, 38, 24);
pbaspect(trajectory_3d_axis, [1 1 0.65]);
pad_3d_axis(trajectory_3d_axis, 0.08);
zlim(trajectory_3d_axis, [70 85]);
zticks(trajectory_3d_axis, sort(unique([zticks(trajectory_3d_axis) 85])));
xlabel(trajectory_3d_axis, 'x [m]');
ylabel(trajectory_3d_axis, 'y [m]');
zlabel(trajectory_3d_axis, 'h [m]');
style_axis(trajectory_3d_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
set(trajectory_3d_axis, 'Position', [0.08 0.18 0.86 0.46]);
add_subplot_label(trajectory_3d_axis, '(b)', figure_font_name, axis_font_size);

add_bottom_legend(trajectory_3d_axis, ...
    [actual_xy_handle reference_xy_handle start_handle end_handle], ...
    {'UAV trajectory', 'Reference trajectory', 'Initial UAV position', 'Final UAV position'}, ...
    figure_font_name, legend_font_size, [0.12 0.045 0.76 0.06]);
export_figure(trajectory_figure, output_directory, 'fig1_obstacle_trajectory_views', export_figures);

%% Figure 2: position tracking
position_figure = make_time_figure([12 13.5]);
[position_x_axis, position_y_axis, position_h_axis, position_ref_handle] = ...
    plot_three_reference_comparison(position_figure, t, ...
    {x, y, h}, {rx, ry, rh}, {'x [m]', 'y [m]', 'h [m]'}, ...
    time_plot_end, actual_color, reference_color, actual_line_width, ...
    reference_line_width, figure_font_name, axis_font_size, axes_line_width, grid_color);
arrange_three_axes_for_bottom_legend(position_x_axis, position_y_axis, position_h_axis);
add_bottom_legend(position_h_axis, position_ref_handle, {'UAV', 'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(position_figure, output_directory, 'fig2_obstacle_positions', export_figures);

%% Figure 3: velocity tracking
velocity_figure = make_time_figure([12 13.5]);
[velocity_x_axis, velocity_y_axis, velocity_h_axis, velocity_ref_handle] = ...
    plot_three_reference_comparison(velocity_figure, t, ...
    {xdot, ydot, hdot}, {rdx, rdy, rdh}, ...
    {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{h}$ [m/s]'}, ...
    time_plot_end, actual_color, reference_color, actual_line_width, ...
    reference_line_width, figure_font_name, axis_font_size, axes_line_width, grid_color);
set_velocity_labels_latex(velocity_x_axis, velocity_y_axis, velocity_h_axis, figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(velocity_x_axis, velocity_y_axis, velocity_h_axis);
add_bottom_legend(velocity_h_axis, velocity_ref_handle, {'UAV', 'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(velocity_figure, output_directory, 'fig3_obstacle_velocities', export_figures);

%% Figure 4: acceleration commands
acceleration_figure = make_time_figure([12 13.5]);
[acc_x_axis, acc_y_axis, acc_h_axis, acc_handles] = ...
    plot_three_reference_comparison_with_limits(acceleration_figure, t, ...
    {xddot, yddot, hddot}, {rd2x, rd2y, rd2h}, a_con, ...
    {'$\ddot{x}$ [m/s$^2$]', '$\ddot{y}$ [m/s$^2$]', '$\ddot{h}$ [m/s$^2$]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
set_velocity_labels_latex(acc_x_axis, acc_y_axis, acc_h_axis, figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(acc_x_axis, acc_y_axis, acc_h_axis);
add_bottom_legend(acc_h_axis, acc_handles, {'UAV', 'reference', 'constraints'}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(acceleration_figure, output_directory, 'fig4_obstacle_accelerations', export_figures);

%% Figure 5: delta acceleration commands
delta_acceleration_figure = make_time_figure([12 13.5]);
[dacc_x_axis, dacc_y_axis, dacc_h_axis, dacc_handles] = ...
    plot_three_reference_comparison_with_limits(delta_acceleration_figure, tk_u, ...
    {dU_d(:,1), dU_d(:,2), dU_d(:,3)}, ...
    {dU_ref(:,1), dU_ref(:,2), dU_ref(:,3)}, da_con, ...
    {'$\Delta\ddot{x}$ [m/s$^2$]', '$\Delta\ddot{y}$ [m/s$^2$]', '$\Delta\ddot{h}$ [m/s$^2$]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
set_velocity_labels_latex(dacc_x_axis, dacc_y_axis, dacc_h_axis, figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(dacc_x_axis, dacc_y_axis, dacc_h_axis);
add_bottom_legend(dacc_h_axis, dacc_handles, {'UAV', 'reference', 'constraints'}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(delta_acceleration_figure, output_directory, 'fig5_obstacle_delta_accelerations', export_figures);

%% Figure 6: Vg, gamma, heading
state_figure = make_time_figure([12 13.5]);
[vg_axis, gamma_axis, psi_axis, state_handles] = ...
    plot_three_reference_comparison_with_limits(state_figure, t, ...
    {Vg, rad2deg(gamma), psi_deg}, {rVg, rad2deg(rGamma), rPsi_deg}, ...
    [C.Vg_min C.Vg_max; rad2deg(C.gamma_min) rad2deg(C.gamma_max); -Inf Inf], ...
    {'V_g [m/s]', '\gamma [deg]', '\psi [deg]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
yticks(vg_axis, sort(unique([yticks(vg_axis) 32])));
gamma_ticks = yticks(gamma_axis);
gamma_ticks(gamma_ticks == 10 | gamma_ticks == 20) = [];
yticks(gamma_axis, gamma_ticks);
psi_ticks = yticks(psi_axis);
psi_ticks(psi_ticks == 200 | psi_ticks == -200) = [];
yticks(psi_axis, sort(unique([psi_ticks -180 180])));
arrange_three_axes_for_bottom_legend(vg_axis, gamma_axis, psi_axis);
add_bottom_legend(psi_axis, state_handles, {'UAV', 'reference', 'constraints'}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(state_figure, output_directory, 'fig6_obstacle_states', export_figures);

%% Figure 7: required UAV inputs
input_figure = make_time_figure([12 13.5]);
[th_axis, ng_axis, phib_axis, input_handles] = ...
    plot_three_reference_comparison_with_limits(input_figure, t, ...
    {Th, ng, phib_deg}, {ref_Th, ref_ng, ref_phib_deg}, ...
    [C.Th_min C.Th_max; C.ng_min C.ng_max; rad2deg(C.phib_min) rad2deg(C.phib_max)], ...
    {'T_h [N]', 'n_g [G]', '\phi_b [deg]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
yticks(th_axis, [1 20 60 120]);
arrange_three_axes_for_bottom_legend(th_axis, ng_axis, phib_axis);
add_bottom_legend(phib_axis, input_handles, {'UAV', 'reference', 'constraints'}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(input_figure, output_directory, 'fig7_obstacle_required_inputs', export_figures);

%% Figure 8: Euclidean distance
ed_figure = make_time_figure([12 7]);
ed_axis = axes('Parent', ed_figure, 'Position', [0.13 0.23 0.80 0.62]);
plot(ed_axis, t, metrics.ED, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
xlabel(ed_axis, 'Time [s]');
ylabel(ed_axis, 'Euclidean Distance [m]');
setup_time_axis(ed_axis, time_plot_end, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
set_nonnegative_y_limits(ed_axis);
export_figure(ed_figure, output_directory, 'fig8_obstacle_euclidean_distance', export_figures);

%% Figure 9: objective function value
solver_figure = make_time_figure([12 7]);
cost_axis = axes('Parent', solver_figure, 'Position', [0.13 0.23 0.80 0.62]);
plot(cost_axis, 1:numel(J), J, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
xlabel(cost_axis, 'k');
ylabel(cost_axis, 'J');
setup_sample_axis(cost_axis, numel(J), figure_font_name, axis_font_size, axes_line_width, grid_color);
set_nonnegative_y_limits(cost_axis);
export_figure(solver_figure, output_directory, 'fig9_obstacle_objective_value', export_figures);

%% Figure 10: Lyapunov terms
lyapunov_figure = make_time_figure([12 13.5]);
[ve_axis, vb_axis, vtotal_axis] = plot_three_single_signals(lyapunov_figure, t, ...
    {logs.Ve, logs.Vb, logs.Vtotal}, {'$V_e(e_k)$', '$V_b(d_i)$', '$V_{BF}(e_k,d_i)$'}, ...
    time_plot_end, actual_color, actual_line_width, figure_font_name, ...
    axis_font_size, axes_line_width, grid_color);
set_lyapunov_labels_latex(ve_axis, vb_axis, vtotal_axis, figure_font_name, axis_font_size);
arrange_three_axes(ve_axis, vb_axis, vtotal_axis);
set_nonnegative_y_limits(ve_axis);
set_nonnegative_y_limits(vb_axis);
set_nonnegative_y_limits(vtotal_axis);
export_figure(lyapunov_figure, output_directory, 'fig10_obstacle_lyapunov_terms', export_figures);

%% Figure 11: obstacle clearance
clearance_figure = make_clearance_figure(t, x, y, h, obstacles, obst_params, ...
    actual_color, limit_color, actual_line_width, limit_line_width, ...
    figure_font_name, axis_font_size, legend_font_size, axes_line_width, grid_color);
export_figure(clearance_figure, output_directory, 'fig11_obstacle_clearance', export_figures);

if export_figures
    fprintf('Obstacle-avoidance thesis figures were saved as SVG files in:\n%s\n', output_directory);
else
    fprintf('Obstacle-avoidance figures were generated without SVG export.\n');
end

%% Local plotting helpers
function fig = make_time_figure(size_cm)
fig = figure('Color', 'w');
set(fig, 'Units', 'centimeters');
fig.Position(3:4) = size_cm;
movegui(fig, 'center');
set(fig, 'PaperPositionMode', 'auto');
end

function [ax1, ax2, ax3, legend_handles] = plot_three_reference_comparison(fig, time_vector, signals, refs, labels, ...
    time_plot_end, actual_color, reference_color, actual_line_width, reference_line_width, ...
    figure_font_name, axis_font_size, axes_line_width, grid_color)
ax1 = subplot(3,1,1, 'Parent', fig);
[actual_handle, ref_handle] = plot_reference_comparison(ax1, time_vector, signals{1}, refs{1}, ...
    actual_color, reference_color, actual_line_width, reference_line_width);
ylabel(ax1, labels{1});
setup_time_axis(ax1, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax1, '(a)', figure_font_name, axis_font_size);

ax2 = subplot(3,1,2, 'Parent', fig);
plot_reference_comparison(ax2, time_vector, signals{2}, refs{2}, ...
    actual_color, reference_color, actual_line_width, reference_line_width);
ylabel(ax2, labels{2});
setup_time_axis(ax2, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax2, '(b)', figure_font_name, axis_font_size);

ax3 = subplot(3,1,3, 'Parent', fig);
plot_reference_comparison(ax3, time_vector, signals{3}, refs{3}, ...
    actual_color, reference_color, actual_line_width, reference_line_width);
ylabel(ax3, labels{3});
xlabel(ax3, 'Time [s]');
setup_time_axis(ax3, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax3, '(c)', figure_font_name, axis_font_size);

legend_handles = [actual_handle ref_handle];
end

function [ax1, ax2, ax3, legend_handles] = plot_three_reference_comparison_with_limits(fig, time_vector, signals, refs, limits, labels, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, reference_line_width, ...
    limit_line_width, figure_font_name, axis_font_size, axes_line_width, grid_color)
ax1 = subplot(3,1,1, 'Parent', fig);
[actual_handle, ref_handle, limit_handle] = plot_reference_comparison_with_limits(ax1, time_vector, ...
    signals{1}, refs{1}, limits(1,:), actual_color, reference_color, limit_color, ...
    actual_line_width, reference_line_width, limit_line_width);
ylabel(ax1, labels{1});
setup_time_axis(ax1, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax1, '(a)', figure_font_name, axis_font_size);

ax2 = subplot(3,1,2, 'Parent', fig);
plot_reference_comparison_with_limits(ax2, time_vector, signals{2}, refs{2}, limits(2,:), ...
    actual_color, reference_color, limit_color, actual_line_width, reference_line_width, limit_line_width);
ylabel(ax2, labels{2});
setup_time_axis(ax2, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax2, '(b)', figure_font_name, axis_font_size);

ax3 = subplot(3,1,3, 'Parent', fig);
plot_reference_comparison_with_limits(ax3, time_vector, signals{3}, refs{3}, limits(3,:), ...
    actual_color, reference_color, limit_color, actual_line_width, reference_line_width, limit_line_width);
ylabel(ax3, labels{3});
xlabel(ax3, 'Time [s]');
setup_time_axis(ax3, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax3, '(c)', figure_font_name, axis_font_size);

legend_handles = [actual_handle ref_handle limit_handle];
end

function [ax1, ax2, ax3] = plot_three_single_signals(fig, time_vector, signals, labels, ...
    time_plot_end, actual_color, actual_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color)
ax1 = subplot(3,1,1, 'Parent', fig);
plot(ax1, time_vector, signals{1}, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
ylabel(ax1, labels{1});
setup_time_axis(ax1, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax1, '(a)', figure_font_name, axis_font_size);

ax2 = subplot(3,1,2, 'Parent', fig);
plot(ax2, time_vector, signals{2}, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
ylabel(ax2, labels{2});
setup_time_axis(ax2, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax2, '(b)', figure_font_name, axis_font_size);

ax3 = subplot(3,1,3, 'Parent', fig);
plot(ax3, time_vector, signals{3}, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
ylabel(ax3, labels{3});
xlabel(ax3, 'Time [s]');
setup_time_axis(ax3, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
add_subplot_label(ax3, '(c)', figure_font_name, axis_font_size);
end

function [actual_handle, reference_handle] = plot_reference_comparison(ax, time_vector, signal, reference, ...
    actual_color, reference_color, actual_line_width, reference_line_width)
actual_handle = plot(ax, time_vector, signal, '-', 'Color', actual_color, ...
    'LineWidth', actual_line_width);
hold(ax, 'on');
reference_handle = plot(ax, time_vector, reference, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
end

function [actual_handle, reference_handle, limit_handle] = plot_reference_comparison_with_limits(ax, time_vector, signal, reference, limits, ...
    actual_color, reference_color, limit_color, actual_line_width, reference_line_width, limit_line_width)
[actual_handle, reference_handle] = plot_reference_comparison(ax, time_vector, signal, reference, ...
    actual_color, reference_color, actual_line_width, reference_line_width);
limit_handle = gobjects(1);
if all(isfinite(limits))
    limit_handle = yline(ax, limits(1), '--', 'Color', limit_color, 'LineWidth', limit_line_width);
    yline(ax, limits(2), '--', 'Color', limit_color, 'LineWidth', limit_line_width);
    add_limit_ticks(ax, limits);
end
end

function setup_time_axis(ax, time_plot_end, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color)
grid(ax, 'off');
xlim(ax, [0 time_plot_end]);
style_axis(ax, figure_font_name, axis_font_size, axes_line_width, grid_color);
pad_y_axis(ax, 0.12);
style_axis_labels(ax, figure_font_name, axis_font_size);
end

function setup_sample_axis(ax, sample_plot_end, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color)
grid(ax, 'off');
xlim(ax, [1 sample_plot_end]);
style_axis(ax, figure_font_name, axis_font_size, axes_line_width, grid_color);
pad_y_axis(ax, 0.12);
style_axis_labels(ax, figure_font_name, axis_font_size);
end

function style_axis(ax, figure_font_name, axis_font_size, axes_line_width, grid_color)
set(ax, 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, ...
    'FontWeight', 'bold', ...
    'LineWidth', axes_line_width, ...
    'GridColor', grid_color, ...
    'GridAlpha', 0.9, ...
    'Box', 'on', ...
    'Layer', 'top', ...
    'TickDir', 'in', ...
    'TickLength', [0.015 0.025], ...
    'XMinorTick', 'on', ...
    'YMinorTick', 'off');
end

function style_axis_labels(ax, figure_font_name, axis_font_size)
xlabel_handle = get(ax, 'XLabel');
ylabel_handle = get(ax, 'YLabel');
set(xlabel_handle, 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
set(ylabel_handle, 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
xlabel_handle.Units = 'normalized';
xlabel_handle.Position(2) = -0.44;
end

function set_velocity_labels_latex(ax1, ax2, ax3, figure_font_name, axis_font_size)
axes_list = [ax1 ax2 ax3];
for i = 1:numel(axes_list)
    ylabel_handle = get(axes_list(i), 'YLabel');
    ylabel_handle.Interpreter = 'latex';
    ylabel_handle.FontName = figure_font_name;
    ylabel_handle.FontSize = axis_font_size;
    ylabel_handle.FontWeight = 'bold';
end
end

function set_lyapunov_labels_latex(ax1, ax2, ax3, figure_font_name, axis_font_size)
axes_list = [ax1 ax2 ax3];
for i = 1:numel(axes_list)
    ylabel_handle = get(axes_list(i), 'YLabel');
    ylabel_handle.Interpreter = 'latex';
    ylabel_handle.FontName = figure_font_name;
    ylabel_handle.FontSize = axis_font_size;
    ylabel_handle.FontWeight = 'bold';
end
end

function arrange_three_axes(ax1, ax2, ax3)
set(ax1, 'Position', [0.13 0.735 0.80 0.17]);
set(ax2, 'Position', [0.13 0.430 0.80 0.17]);
set(ax3, 'Position', [0.13 0.155 0.80 0.17]);
end

function arrange_three_axes_for_bottom_legend(ax1, ax2, ax3)
set(ax1, 'Position', [0.13 0.750 0.80 0.17]);
set(ax2, 'Position', [0.13 0.470 0.80 0.17]);
set(ax3, 'Position', [0.13 0.220 0.80 0.17]);
end

function add_subplot_label(ax, label_text, figure_font_name, axis_font_size)
text(ax, 0.52, -0.20, label_text, 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontName', figure_font_name, 'FontSize', axis_font_size, ...
    'FontWeight', 'bold', 'Clipping', 'off');
end

function add_subplot_label_at(ax, label_text, figure_font_name, axis_font_size, x_pos, y_pos)
text(ax, x_pos, y_pos, label_text, 'Units', 'normalized', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontName', figure_font_name, 'FontSize', axis_font_size, ...
    'FontWeight', 'bold', 'Clipping', 'off');
end

function legend_handle = add_bottom_legend(ax, handles, labels, figure_font_name, ...
    legend_font_size, legend_position)
handles = handles(isgraphics(handles));
legend_handle = legend(ax, handles, labels(1:numel(handles)), 'Orientation', 'horizontal', ...
    'FontName', figure_font_name, 'FontSize', legend_font_size, ...
    'FontWeight', 'bold', 'Box', 'off');
set(legend_handle, 'Units', 'normalized', 'Position', legend_position);
end

function pad_y_axis(ax, padding_fraction)
current_ylim = ylim(ax);
y_range = diff(current_ylim);
if y_range == 0
    y_range = max(1, abs(current_ylim(1)));
end
y_padding = padding_fraction * y_range;
ylim(ax, [current_ylim(1) - y_padding, current_ylim(2) + y_padding]);
end

function set_nonnegative_y_limits(ax)
current_ylim = ylim(ax);
y_max = current_ylim(2);
if y_max <= 0
    y_max = 1;
end
ylim(ax, [0 y_max]);
end

function pad_xy_axis(ax, padding_fraction)
current_xlim = xlim(ax);
current_ylim = ylim(ax);
x_range = diff(current_xlim);
y_range = diff(current_ylim);
if x_range == 0
    x_range = max(1, abs(current_xlim(1)));
end
if y_range == 0
    y_range = max(1, abs(current_ylim(1)));
end
xlim(ax, current_xlim + [-1 1] * padding_fraction * x_range);
ylim(ax, current_ylim + [-1 1] * padding_fraction * y_range);
end

function pad_3d_axis(ax, padding_fraction)
current_xlim = xlim(ax);
current_ylim = ylim(ax);
current_zlim = zlim(ax);
x_range = diff(current_xlim);
y_range = diff(current_ylim);
z_range = diff(current_zlim);
if x_range == 0
    x_range = max(1, abs(current_xlim(1)));
end
if y_range == 0
    y_range = max(1, abs(current_ylim(1)));
end
if z_range == 0
    z_range = max(1, abs(current_zlim(1)));
end
xlim(ax, current_xlim + [-1 1] * padding_fraction * x_range);
ylim(ax, current_ylim + [-1 1] * padding_fraction * y_range);
zlim(ax, current_zlim + [-1 1] * padding_fraction * z_range);
end

function add_limit_ticks(ax, limits)
if any(~isfinite(limits))
    return;
end
current_ticks = yticks(ax);
current_ylim = ylim(ax);
close_to_limit = false(size(current_ticks));
for i = 1:numel(limits)
    close_to_limit = close_to_limit | ...
        (abs(current_ticks - limits(i)) < 0.08 * diff(current_ylim) & ...
        abs(current_ticks - limits(i)) > eps);
end
current_ticks(close_to_limit) = [];
new_ticks = unique([current_ticks(:); limits(:)]);
new_ticks = new_ticks(new_ticks >= min(current_ylim) & new_ticks <= max(current_ylim));
yticks(ax, new_ticks);
end

function plot_obstacle_centers_2d(ax, obstacles)
for i = 1:numel(obstacles)
    plot(ax, obstacles(i).pos(1), obstacles(i).pos(2), 'x', ...
        'Color', [0.25 0.25 0.25], 'MarkerSize', 5, 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
end
end

function plot_obstacle_centers_3d(ax, obstacles)
for i = 1:numel(obstacles)
    plot3(ax, obstacles(i).pos(1), obstacles(i).pos(2), obstacles(i).pos(3), 'x', ...
        'Color', [0.25 0.25 0.25], 'MarkerSize', 5, 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
end
end

function fig = make_clearance_figure(t, x, y, h, obstacles, obst_params, actual_color, ...
    limit_color, actual_line_width, limit_line_width, figure_font_name, ...
    axis_font_size, legend_font_size, axes_line_width, grid_color)
if isfield(obst_params, 'obstacles_active_until')
    obst_plot_mask = t <= obst_params.obstacles_active_until;
else
    obst_plot_mask = true(size(t));
end
t_obst_plot = t(obst_plot_mask);

obst_dist = zeros(numel(t_obst_plot), numel(obstacles));
for i = 1:numel(obstacles)
    dx_obst = x(obst_plot_mask) - obstacles(i).pos(1);
    dy_obst = y(obst_plot_mask) - obstacles(i).pos(2);
    dh_obst = h(obst_plot_mask) - obstacles(i).pos(3);
    obst_dist(:,i) = sqrt(dx_obst.^2 + dy_obst.^2 + dh_obst.^2);
end

collision_bound = obst_params.r_uav + obst_params.r_obst;
obst_clearance = obst_dist - collision_bound;
min_obst_clearance = min(obst_clearance, [], 2);

fig = figure('Color', 'w');
set(fig, 'Units', 'centimeters');
fig.Position(3:4) = [16 16];
movegui(fig, 'center');
set(fig, 'PaperPositionMode', 'auto');

main_axis = subplot(3,2,[1 2], 'Parent', fig);
clearance_handle = plot(main_axis, t_obst_plot, min_obst_clearance, '-', ...
    'Color', actual_color, 'LineWidth', actual_line_width);
hold(main_axis, 'on');
limit_handle = yline(main_axis, 0, '--', 'Color', limit_color, 'LineWidth', limit_line_width);
xlabel(main_axis, 'Time [s]');
ylabel(main_axis, 'Clearance [m]');
setup_time_axis(main_axis, t_obst_plot(end), figure_font_name, axis_font_size, axes_line_width, grid_color);
ylim(main_axis, [min(-0.5, min(min_obst_clearance)-0.2), max(min_obst_clearance)+5]);
set(main_axis, 'Position', [0.13 0.73 0.80 0.17]);
title(main_axis, 'Minimum clearance between the UAV and any obstacle', ...
    'FontName', figure_font_name, 'FontSize', axis_font_size, ...
    'FontWeight', 'bold');
main_xlabel = get(main_axis, 'XLabel');
main_xlabel.Position(2) = -0.30;
add_subplot_label(main_axis, '(a)', figure_font_name, axis_font_size);

num_zoom_obstacles = min(4, numel(obstacles));
for i = 1:num_zoom_obstacles
    obstacle_clearance = obst_clearance(:,i);
    zoom_idx = find(obstacle_clearance <= 1);
    near_collision_zoom = ~isempty(zoom_idx);
    if isempty(zoom_idx)
        [~, min_clearance_idx] = min(obstacle_clearance);
        zoom_idx = max(1, min_clearance_idx-20):min(numel(t_obst_plot), min_clearance_idx+20);
    end

    if i == 1
        zoom_time_min = 30.0;
        zoom_time_max = 30.6;
    else
        zoom_time_min = max(t_obst_plot(1), t_obst_plot(zoom_idx(1)) - 0.5);
        zoom_time_max = min(t_obst_plot(end), t_obst_plot(zoom_idx(end)) + 0.5);
    end
    zoom_mask = (t_obst_plot >= zoom_time_min) & (t_obst_plot <= zoom_time_max);

    zoom_axis = subplot(3,2,i+2, 'Parent', fig);
    if i == 1
        set(zoom_axis, 'Position', [0.13 0.45 0.34 0.16]);
    elseif i == 2
        set(zoom_axis, 'Position', [0.59 0.45 0.34 0.16]);
    elseif i == 3
        set(zoom_axis, 'Position', [0.13 0.18 0.34 0.16]);
    else
        set(zoom_axis, 'Position', [0.59 0.18 0.34 0.16]);
    end
    plot(zoom_axis, t_obst_plot, obstacle_clearance, '-', ...
        'Color', actual_color, 'LineWidth', actual_line_width);
    hold(zoom_axis, 'on');
    yline(zoom_axis, 0, '--', 'Color', limit_color, 'LineWidth', limit_line_width);
    xlim(zoom_axis, [zoom_time_min, zoom_time_max]);
    zoom_clearance_min = min(obstacle_clearance(zoom_mask));
    zoom_clearance_max = max(obstacle_clearance(zoom_mask));
    if near_collision_zoom
        if zoom_clearance_min >= 0
            zoom_y_min = 0;
        else
            zoom_y_min = zoom_clearance_min - 0.05;
        end
        ylim(zoom_axis, [zoom_y_min, ...
            max(1, min(1.2, zoom_clearance_max + 0.1))]);
    else
        zoom_clearance_margin = max(0.1, 0.05 * (zoom_clearance_max - zoom_clearance_min));
        if zoom_clearance_min >= 0
            zoom_y_min = 0;
        else
            zoom_y_min = zoom_clearance_min - zoom_clearance_margin;
        end
        ylim(zoom_axis, [zoom_y_min, ...
            zoom_clearance_max + zoom_clearance_margin]);
    end
    xlabel(zoom_axis, 'Time [s]');
    ylabel(zoom_axis, 'Clearance [m]');
    style_axis(zoom_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
    style_axis_labels(zoom_axis, figure_font_name, axis_font_size);
    zoom_xlabel = get(zoom_axis, 'XLabel');
    zoom_xlabel.Position(2) = -0.34;
    if isfield(obstacles, 'id')
        obstacle_id = obstacles(i).id;
    else
        obstacle_id = i;
    end
    add_subplot_label(zoom_axis, sprintf('(%c)', char('a' + i)), figure_font_name, axis_font_size);
    title(zoom_axis, sprintf('Obstacle %d', obstacle_id), ...
        'FontName', figure_font_name, 'FontSize', axis_font_size, ...
        'FontWeight', 'bold');
end

legend_handle = legend(main_axis, [clearance_handle limit_handle], ...
    {'clearance', 'limit'}, 'Orientation', 'horizontal', ...
    'FontName', figure_font_name, 'FontSize', legend_font_size, ...
    'FontWeight', 'bold', 'Box', 'off');
set(legend_handle, 'Units', 'normalized', 'Position', [0.37 0.045 0.26 0.05]);

end

function export_figure(fig, output_directory, file_stem, export_figures)
if ~export_figures
    return;
end
print(fig, fullfile(output_directory, [file_stem '.svg']), ...
    '-dsvg', '-painters');
end
