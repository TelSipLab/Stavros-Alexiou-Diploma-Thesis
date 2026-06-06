clc; clear; close all;

%% Reference trajectory setup
T = 220;
Ts = 0.1;
t = (0:Ts:T).';
N = numel(t);
export_figures = true;

params = uav_params();
C = uav_constraints();

% Acceleration and delta-acceleration constraints
a_con = [-5 5; -5 5; -5 5];
da_con = [-1 1; -1 1; -1 1];

% Wind value used only for the required UAV inputs along the reference
z_ref = 80;
Vw_ref = 0.215 * params.Vmw * log10(z_ref) + 0.285 * params.Vmw;

%% Paper style setup
figure_font_name = 'Times New Roman';
axis_font_size = 10;
legend_font_size = 9;
title_font_size = 11;
bottom_legend_position = [0.37 0.035 0.26 0.05];

reference_line_width = 1.8;
limit_line_width = 1.5;
axes_line_width = 1.1;

reference_color = [0.00 0.45 0.00];
limit_color = [1.00 0.20 0.20];
grid_color = [0.85 0.85 0.85];

output_directory = fullfile(fileparts(mfilename('fullpath')), ...
    'reference_trajectory_figures_thesis');
if export_figures && ~exist(output_directory, 'dir')
    mkdir(output_directory);
end
if export_figures
    old_svg_files = dir(fullfile(output_directory, '*.svg'));
    for i = 1:numel(old_svg_files)
        delete(fullfile(output_directory, old_svg_files(i).name));
    end
end

%% Reference trajectory signals
ref = zeros(N,9);
for i = 1:N
    ref(i,:) = ref_state_hippodrome(t(i)).';
end

rx = ref(:,1); ry = ref(:,2); rh = ref(:,3);
rdx = ref(:,4); rdy = ref(:,5); rdh = ref(:,6);
rd2x = ref(:,7); rd2y = ref(:,8); rd2h = ref(:,9);

rVg = sqrt(rdx.^2 + rdy.^2 + rdh.^2);
rGamma = asin(rdh ./ rVg);
rPsi = atan2(rdy, rdx);
rPsi_deg = mod(rad2deg(rPsi) + 180, 360) - 180;

dA_ref = zeros(N,3);
dA_ref(1,:) = ref(1,7:9);
dA_ref(2:end,:) = ref(2:end,7:9) - ref(1:end-1,7:9);

rPhib = zeros(N,1);
rNg = zeros(N,1);
rTh = zeros(N,1);
for i = 1:N
    [rPhib(i), rNg(i), rTh(i), ~] = di_mapping( ...
        rd2x(i), rd2y(i), rd2h(i), rPsi(i), rGamma(i), rVg(i), Vw_ref, params);
end

%% Figure 1: reference trajectory views
trajectory_figure = figure('Color', 'w');
set(trajectory_figure, 'Units', 'centimeters');
trajectory_figure.Position(3:4) = [12 15];
set(trajectory_figure, 'PaperPositionMode', 'auto');

trajectory_xy_axis = subplot(2,1,1, 'Parent', trajectory_figure);
reference_handle = plot(trajectory_xy_axis, rx, ry, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
grid(trajectory_xy_axis, 'off');
axis(trajectory_xy_axis, 'tight');
axis(trajectory_xy_axis, 'equal');
pad_xy_axis(trajectory_xy_axis, 0.08);
xlabel(trajectory_xy_axis, 'x [m]', 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
ylabel(trajectory_xy_axis, 'y [m]', 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
style_axis(trajectory_xy_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
set(trajectory_xy_axis, 'Position', [0.13 0.67 0.80 0.20]);
add_subplot_label_at(trajectory_xy_axis, '(a)', figure_font_name, axis_font_size, 0.52, -0.52);

trajectory_3d_axis = subplot(2,1,2, 'Parent', trajectory_figure);
plot3(trajectory_3d_axis, rx, ry, rh, '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
grid(trajectory_3d_axis, 'off');
axis(trajectory_3d_axis, 'tight');
view(trajectory_3d_axis, 38, 24);
pbaspect(trajectory_3d_axis, [1 1 0.65]);
pad_3d_axis(trajectory_3d_axis, 0.08);
xlabel(trajectory_3d_axis, 'x [m]', 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
ylabel(trajectory_3d_axis, 'y [m]', 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
zlabel(trajectory_3d_axis, 'h [m]', 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
style_axis(trajectory_3d_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
set(trajectory_3d_axis, 'Position', [0.13 0.20 0.80 0.38]);
add_subplot_label(trajectory_3d_axis, '(b)', figure_font_name, axis_font_size);

add_bottom_legend(trajectory_3d_axis, reference_handle, {'reference trajectory'}, ...
    figure_font_name, legend_font_size, [0.39 0.055 0.24 0.05]);
export_figure(trajectory_figure, output_directory, 'fig1_reference_trajectory_views', export_figures);

%% Figure 2: reference positions
position_figure = make_time_figure([12 13.5]);

position_x_axis = subplot(3,1,1, 'Parent', position_figure);
position_reference_handle = plot(position_x_axis, t, rx, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(position_x_axis, 'x [m]');
setup_time_axis(position_x_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(position_x_axis, '(a)', figure_font_name, axis_font_size);

position_y_axis = subplot(3,1,2, 'Parent', position_figure);
plot(position_y_axis, t, ry, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(position_y_axis, 'y [m]');
setup_time_axis(position_y_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(position_y_axis, '(b)', figure_font_name, axis_font_size);

position_h_axis = subplot(3,1,3, 'Parent', position_figure);
plot(position_h_axis, t, rh, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(position_h_axis, 'h [m]');
xlabel(position_h_axis, 'Time [s]');
setup_time_axis(position_h_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(position_h_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(position_x_axis, position_y_axis, position_h_axis);
add_bottom_legend(position_h_axis, position_reference_handle, {'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(position_figure, output_directory, 'fig2_reference_positions', export_figures);

%% Figure 3: reference velocities
velocity_figure = make_time_figure([12 13.5]);

velocity_x_axis = subplot(3,1,1, 'Parent', velocity_figure);
velocity_reference_handle = plot(velocity_x_axis, t, rdx, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(velocity_x_axis, '$\dot{x}$ [m/s]', 'Interpreter', 'latex');
setup_time_axis(velocity_x_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(velocity_x_axis, '(a)', figure_font_name, axis_font_size);

velocity_y_axis = subplot(3,1,2, 'Parent', velocity_figure);
plot(velocity_y_axis, t, rdy, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(velocity_y_axis, '$\dot{y}$ [m/s]', 'Interpreter', 'latex');
setup_time_axis(velocity_y_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(velocity_y_axis, '(b)', figure_font_name, axis_font_size);

velocity_h_axis = subplot(3,1,3, 'Parent', velocity_figure);
plot(velocity_h_axis, t, rdh, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(velocity_h_axis, '$\dot{h}$ [m/s]', 'Interpreter', 'latex');
xlabel(velocity_h_axis, 'Time [s]');
setup_time_axis(velocity_h_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(velocity_h_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(velocity_x_axis, velocity_y_axis, velocity_h_axis);
add_bottom_legend(velocity_h_axis, velocity_reference_handle, {'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(velocity_figure, output_directory, 'fig3_reference_velocities', export_figures);

%% Figure 4: reference accelerations
acceleration_figure = make_time_figure([12 13.5]);

acceleration_x_axis = subplot(3,1,1, 'Parent', acceleration_figure);
acceleration_reference_handle = plot(acceleration_x_axis, t, rd2x, '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(acceleration_x_axis, '$\ddot{x}$ [m/s$^2$]', 'Interpreter', 'latex');
setup_time_axis(acceleration_x_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(acceleration_x_axis, '(a)', figure_font_name, axis_font_size);

acceleration_y_axis = subplot(3,1,2, 'Parent', acceleration_figure);
plot(acceleration_y_axis, t, rd2y, '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(acceleration_y_axis, '$\ddot{y}$ [m/s$^2$]', 'Interpreter', 'latex');
setup_time_axis(acceleration_y_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(acceleration_y_axis, '(b)', figure_font_name, axis_font_size);

acceleration_h_axis = subplot(3,1,3, 'Parent', acceleration_figure);
plot(acceleration_h_axis, t, rd2h, '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(acceleration_h_axis, '$\ddot{h}$ [m/s$^2$]', 'Interpreter', 'latex');
xlabel(acceleration_h_axis, 'Time [s]');
setup_time_axis(acceleration_h_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(acceleration_h_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(acceleration_x_axis, acceleration_y_axis, acceleration_h_axis);
add_bottom_legend(acceleration_h_axis, acceleration_reference_handle, ...
    {'reference'}, figure_font_name, legend_font_size, bottom_legend_position);
export_figure(acceleration_figure, output_directory, 'fig4_reference_accelerations', export_figures);

%% Figure 5: reference delta accelerations
delta_acceleration_figure = make_time_figure([12 13.5]);

delta_acceleration_x_axis = subplot(3,1,1, 'Parent', delta_acceleration_figure);
delta_acceleration_reference_handle = plot(delta_acceleration_x_axis, t, dA_ref(:,1), '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(delta_acceleration_x_axis, '$\Delta\ddot{x}$ [m/s$^2$]', 'Interpreter', 'latex');
setup_time_axis(delta_acceleration_x_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(delta_acceleration_x_axis, '(a)', figure_font_name, axis_font_size);

delta_acceleration_y_axis = subplot(3,1,2, 'Parent', delta_acceleration_figure);
plot(delta_acceleration_y_axis, t, dA_ref(:,2), '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(delta_acceleration_y_axis, '$\Delta\ddot{y}$ [m/s$^2$]', 'Interpreter', 'latex');
setup_time_axis(delta_acceleration_y_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(delta_acceleration_y_axis, '(b)', figure_font_name, axis_font_size);

delta_acceleration_h_axis = subplot(3,1,3, 'Parent', delta_acceleration_figure);
plot(delta_acceleration_h_axis, t, dA_ref(:,3), '--', ...
    'Color', reference_color, 'LineWidth', reference_line_width);
ylabel(delta_acceleration_h_axis, '$\Delta\ddot{h}$ [m/s$^2$]', 'Interpreter', 'latex');
xlabel(delta_acceleration_h_axis, 'Time [s]');
setup_time_axis(delta_acceleration_h_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(delta_acceleration_h_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(delta_acceleration_x_axis, delta_acceleration_y_axis, delta_acceleration_h_axis);
add_bottom_legend(delta_acceleration_h_axis, delta_acceleration_reference_handle, ...
    {'reference'}, figure_font_name, legend_font_size, bottom_legend_position);
export_figure(delta_acceleration_figure, output_directory, 'fig5_reference_delta_accelerations', export_figures);

%% Figure 6: reference Vg, gamma, heading
state_figure = make_time_figure([12 13.5]);

ground_speed_axis = subplot(3,1,1, 'Parent', state_figure);
[state_reference_handle, state_limit_handle] = plot_signal_with_limits(ground_speed_axis, t, rVg, [C.Vg_min C.Vg_max], ...
    reference_color, limit_color, reference_line_width, limit_line_width);
ylabel(ground_speed_axis, 'V_g [m/s]');
setup_time_axis(ground_speed_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
yticks(ground_speed_axis, sort(unique([yticks(ground_speed_axis) 32])));
add_subplot_label(ground_speed_axis, '(a)', figure_font_name, axis_font_size);

flight_path_axis = subplot(3,1,2, 'Parent', state_figure);
plot_signal_with_limits(flight_path_axis, t, rad2deg(rGamma), ...
    rad2deg([C.gamma_min C.gamma_max]), reference_color, limit_color, ...
    reference_line_width, limit_line_width);
ylabel(flight_path_axis, '\gamma [deg]');
setup_time_axis(flight_path_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
gamma_ticks = yticks(flight_path_axis);
gamma_ticks(gamma_ticks == 10 | gamma_ticks == 20) = [];
yticks(flight_path_axis, gamma_ticks);
add_subplot_label(flight_path_axis, '(b)', figure_font_name, axis_font_size);

heading_axis = subplot(3,1,3, 'Parent', state_figure);
plot(heading_axis, t, rPsi_deg, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
ylabel(heading_axis, '\psi [deg]');
xlabel(heading_axis, 'Time [s]');
setup_time_axis(heading_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
psi_ticks = yticks(heading_axis);
psi_ticks(psi_ticks == 200) = [];
psi_ticks(psi_ticks == -200) = [];
yticks(heading_axis, sort(unique([psi_ticks -180 180])));
add_subplot_label(heading_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(ground_speed_axis, flight_path_axis, heading_axis);
add_bottom_legend(heading_axis, [state_reference_handle state_limit_handle], ...
    {'reference', 'limits'}, figure_font_name, legend_font_size, bottom_legend_position);
export_figure(state_figure, output_directory, 'fig6_reference_states', export_figures);

%% Figure 7: required UAV inputs for reference tracking
input_figure = make_time_figure([12 13.5]);

thrust_axis = subplot(3,1,1, 'Parent', input_figure);
[input_reference_handle, input_limit_handle] = plot_signal_with_limits(thrust_axis, t, rTh, [C.Th_min C.Th_max], ...
    reference_color, limit_color, reference_line_width, limit_line_width);
ylabel(thrust_axis, 'T_h [N]');
setup_time_axis(thrust_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
yticks(thrust_axis, [1 20 60 120]);
add_subplot_label(thrust_axis, '(a)', figure_font_name, axis_font_size);

g_load_axis = subplot(3,1,2, 'Parent', input_figure);
plot_signal_with_limits(g_load_axis, t, rNg, [C.ng_min C.ng_max], ...
    reference_color, limit_color, reference_line_width, limit_line_width);
ylabel(g_load_axis, 'n_g [G]');
setup_time_axis(g_load_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(g_load_axis, '(b)', figure_font_name, axis_font_size);

bank_angle_axis = subplot(3,1,3, 'Parent', input_figure);
plot_signal_with_limits(bank_angle_axis, t, rad2deg(rPhib), ...
    rad2deg([C.phib_min C.phib_max]), reference_color, limit_color, ...
    reference_line_width, limit_line_width);
ylabel(bank_angle_axis, '\phi_b [deg]');
xlabel(bank_angle_axis, 'Time [s]');
setup_time_axis(bank_angle_axis, T, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size);
add_subplot_label(bank_angle_axis, '(c)', figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(thrust_axis, g_load_axis, bank_angle_axis);
input_legend = legend(bank_angle_axis, [input_reference_handle input_limit_handle], ...
    {'reference', 'limits'}, 'Orientation', 'horizontal', ...
    'FontName', figure_font_name, 'FontSize', legend_font_size, ...
    'FontWeight', 'bold', 'Box', 'off');
set(input_legend, 'Units', 'normalized', 'Position', bottom_legend_position);
export_figure(input_figure, output_directory, 'fig7_required_uav_inputs', export_figures);

if export_figures
    fprintf('Reference trajectory thesis figures were saved as SVG files in:\n%s\n', output_directory);
else
    fprintf('Reference trajectory figures were generated without SVG export.\n');
end

%% Local plotting helpers
function fig = make_time_figure(size_cm)
fig = figure('Color', 'w');
set(fig, 'Units', 'centimeters');
fig.Position(3:4) = size_cm;
set(fig, 'PaperPositionMode', 'auto');
end

function [reference_handle, limit_handle] = plot_signal_with_limits(ax, time_vector, signal, limits, reference_color, ...
    limit_color, reference_line_width, limit_line_width)
reference_handle = plot(ax, time_vector, signal, '--', 'Color', reference_color, ...
    'LineWidth', reference_line_width);
hold(ax, 'on');
limit_handle = yline(ax, limits(1), '--', 'Color', limit_color, 'LineWidth', limit_line_width);
yline(ax, limits(2), '--', 'Color', limit_color, 'LineWidth', limit_line_width);
add_limit_ticks(ax, limits);
end

function setup_time_axis(ax, time_plot_end, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color, title_font_size)
grid(ax, 'off');
xlim(ax, [0 time_plot_end]);
style_axis(ax, figure_font_name, axis_font_size, axes_line_width, grid_color);
pad_y_axis(ax, 0.12);
title_handle = get(ax, 'Title');
set(title_handle, 'FontName', figure_font_name, ...
    'FontSize', title_font_size, 'FontWeight', 'bold');
xlabel_handle = get(ax, 'XLabel');
ylabel_handle = get(ax, 'YLabel');
set(xlabel_handle, 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
set(ylabel_handle, 'FontName', figure_font_name, ...
    'FontSize', axis_font_size, 'FontWeight', 'bold');
xlabel_handle.Units = 'normalized';
xlabel_handle.Position(2) = -0.44;
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

function pad_y_axis(ax, padding_fraction)
current_ylim = ylim(ax);
y_range = diff(current_ylim);
if y_range == 0
    y_range = max(1, abs(current_ylim(1)));
end
y_padding = padding_fraction * y_range;
ylim(ax, [current_ylim(1) - y_padding, current_ylim(2) + y_padding]);
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
x_padding = padding_fraction * x_range;
y_padding = padding_fraction * y_range;
xlim(ax, [current_xlim(1) - x_padding, current_xlim(2) + x_padding]);
ylim(ax, [current_ylim(1) - y_padding, current_ylim(2) + y_padding]);
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
x_padding = padding_fraction * x_range;
y_padding = padding_fraction * y_range;
z_padding = padding_fraction * z_range;
xlim(ax, [current_xlim(1) - x_padding, current_xlim(2) + x_padding]);
ylim(ax, [current_ylim(1) - y_padding, current_ylim(2) + y_padding]);
zlim(ax, [current_zlim(1) - z_padding, current_zlim(2) + z_padding]);
end

function legend_handle = add_reference_legend(ax, figure_font_name, legend_font_size, legend_location)
legend_handle = legend(ax, {'reference', 'limits'}, 'Location', legend_location, ...
    'FontName', figure_font_name, 'FontSize', legend_font_size, ...
    'FontWeight', 'bold', 'Box', 'on');
end

function add_subplot_label(ax, label_text, figure_font_name, axis_font_size)
text(ax, 0.52, -0.16, label_text, 'Units', 'normalized', ...
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

function legend_handle = add_bottom_legend(ax, handles, labels, figure_font_name, ...
    legend_font_size, legend_position)
legend_handle = legend(ax, handles, labels, 'Orientation', 'horizontal', ...
    'FontName', figure_font_name, 'FontSize', legend_font_size, ...
    'FontWeight', 'bold', 'Box', 'off');
set(legend_handle, 'Units', 'normalized', 'Position', legend_position);
end

function add_limit_ticks(ax, limits)
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

% Previous per-subplot legend style kept for easy restoration.
% Example:
%   add_reference_legend(ax, figure_font_name, legend_font_size, 'northeast');
function export_figure(fig, output_directory, file_stem, export_figures)
if ~export_figures
    return;
end
print(fig, fullfile(output_directory, [file_stem '.svg']), ...
    '-dsvg', '-painters');
end
