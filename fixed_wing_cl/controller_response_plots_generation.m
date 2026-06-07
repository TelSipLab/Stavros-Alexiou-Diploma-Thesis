clc; close all;

%% Controller response workspaces
export_figures = true;
selected_controller_ids = {};

script_directory = fileparts(mfilename('fullpath'));
output_root = fullfile(script_directory, 'controller_response_figures_thesis');

controller_cases = [ ...
    struct('id', 'contractive_mpc', 'name', 'Contractive MPC', ...
    'patterns', {{'contractive_mpc*.mat', '*contractive*mpc*.mat', '*mpc*.mat'}}); ...
    struct('id', 'state_feedback', 'name', 'State feedback', ...
    'patterns', {{'state_feedback*.mat', '*state*feedback*.mat', 'sf*.mat', '*sf*.mat'}}); ...
    struct('id', 'pid', 'name', 'PID', ...
    'patterns', {{'pid_ff_run.mat', 'pid_ff*.mat', '*pid_ff*.mat'}}) ...
];

if ~exist(output_root, 'dir')
    mkdir(output_root);
end

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

C = uav_constraints();

%% Generate figures for each saved controller run
processed_cases = 0;
for case_idx = 1:numel(controller_cases)
    case_data = controller_cases(case_idx);
    if ~isempty(selected_controller_ids) && ~any(strcmp(case_data.id, selected_controller_ids))
        continue;
    end
    case_workspace_file = find_controller_workspace(script_directory, case_data.patterns);
    case_output_directory = fullfile(output_root, case_data.id);
    if ~exist(case_output_directory, 'dir')
        mkdir(case_output_directory);
    end

    if isempty(case_workspace_file)
        fprintf('No workspace found for %s in:\n%s\nExpected one of these patterns: %s\n\n', ...
            case_data.name, script_directory, strjoin(case_data.patterns, ', '));
        continue;
    end

    if export_figures
        old_svg_files = dir(fullfile(case_output_directory, '*.svg'));
        for i = 1:numel(old_svg_files)
            delete(fullfile(case_output_directory, old_svg_files(i).name));
        end
    end

    run_data = load(case_workspace_file);
    validate_run_data(run_data, case_data.name);
    generate_controller_figures(run_data, case_data, case_output_directory, export_figures, ...
        C, figure_font_name, axis_font_size, legend_font_size, bottom_legend_position, ...
        actual_line_width, reference_line_width, limit_line_width, axes_line_width, ...
        actual_color, reference_color, limit_color, start_color, end_color, grid_color);
    processed_cases = processed_cases + 1;
end

if processed_cases == 0
    fprintf(['No controller figures were generated yet.\n' ...
        'Save the three controller workspaces directly in:\n' ...
        '  %s\n' ...
        'Suggested names:\n' ...
        '  contractive_mpc_run.mat\n' ...
        '  state_feedback_run.mat\n' ...
        '  pid_ff_run.mat\n'], script_directory);
elseif export_figures
    fprintf('Controller response thesis figures were saved as SVG files in:\n%s\n', output_root);
else
    fprintf('Controller response figures were generated without SVG export.\n');
end

%% Local workflow helpers
function workspace_file = find_controller_workspace(script_directory, patterns)
candidate_files = [];
for i = 1:numel(patterns)
    candidate_files = [candidate_files; dir(fullfile(script_directory, patterns{i}))]; %#ok<AGROW>
end
if isempty(candidate_files)
    workspace_file = '';
    return;
end
[~, newest_idx] = max([candidate_files.datenum]);
workspace_file = fullfile(candidate_files(newest_idx).folder, candidate_files(newest_idx).name);
end

function validate_run_data(run_data, case_name)
required_fields = {'t_total', 'logs', 'metrics'};
for i = 1:numel(required_fields)
    if ~isfield(run_data, required_fields{i})
        error('Workspace for %s is missing required variable "%s".', ...
            case_name, required_fields{i});
    end
end
end

function generate_controller_figures(run_data, case_data, output_directory, export_figures, C, ...
    figure_font_name, axis_font_size, legend_font_size, bottom_legend_position, ...
    actual_line_width, reference_line_width, limit_line_width, axes_line_width, ...
    actual_color, reference_color, limit_color, start_color, end_color, grid_color)

t = run_data.t_total;
logs = run_data.logs;
metrics = run_data.metrics;
time_plot_end = t(end);
if strcmp(case_data.id, 'contractive_mpc')
    bounds_label = 'constraints';
else
    bounds_label = 'limits';
end

x = logs.x; y = logs.y; h = logs.h;
Vg = logs.Vg; gamma = logs.gamma; psi = logs.psi;
xdot = logs.xdot; ydot = logs.ydot; hdot = logs.hdot;

rx = logs.ref_cont(:,1); ry = logs.ref_cont(:,2); rh = logs.ref_cont(:,3);
rdx = logs.ref_vel_cont(:,1); rdy = logs.ref_vel_cont(:,2); rdh = logs.ref_vel_cont(:,3);
rVg = sqrt(rdx.^2 + rdy.^2 + rdh.^2);
rGamma = asin(rdh ./ rVg);
rPsi = atan2(rdy, rdx);
psi_deg = mod(rad2deg(psi) + 180, 360) - 180;
rPsi_deg = mod(rad2deg(rPsi) + 180, 360) - 180;

xddot = logs.U(:,1);
yddot = logs.U(:,2);
hddot = logs.U(:,3);
Th = logs.Th_d_zoh;
ng = logs.ng_d_zoh;
phib_deg = rad2deg(logs.phib_d_zoh);

[ref_acc_cont, ref_acc_samp] = get_reference_accelerations(t, logs.tk);
rd2x = ref_acc_cont(:,1); rd2y = ref_acc_cont(:,2); rd2h = ref_acc_cont(:,3);
[ref_Th, ref_ng, ref_phib_deg] = get_reference_inputs(ref_acc_cont, ...
    logs.ref_vel_cont, logs.Vw, run_data.params);

tk_u = logs.tk(1:end-1).';
U_d = logs.U_d;
dU_d = zeros(size(U_d));
dU_ref = zeros(size(ref_acc_samp));
dU_d(1,:) = U_d(1,:);
dU_d(2:end,:) = U_d(2:end,:) - U_d(1:end-1,:);
dU_ref(1,:) = ref_acc_samp(1,:);
dU_ref(2:end,:) = ref_acc_samp(2:end,:) - ref_acc_samp(1:end-1,:);

[a_con, da_con] = get_acceleration_constraints(run_data, U_d);

%% Figure 1: trajectory views
trajectory_figure = figure('Color', 'w', 'Name', [case_data.name ' trajectory']);
set(trajectory_figure, 'Units', 'centimeters');
trajectory_figure.Position(3:4) = [16 18];
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
set(trajectory_xy_axis, 'Position', [0.13 0.735 0.80 0.16]);
add_subplot_label_at(trajectory_xy_axis, '(a)', figure_font_name, axis_font_size, 0.52, -0.42);

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
xlabel(trajectory_3d_axis, 'x [m]');
ylabel(trajectory_3d_axis, 'y [m]');
zlabel(trajectory_3d_axis, 'h [m]');
style_axis(trajectory_3d_axis, figure_font_name, axis_font_size, axes_line_width, grid_color);
set(trajectory_3d_axis, 'Position', [0.10 0.225 0.82 0.38]);
add_subplot_label_at(trajectory_3d_axis, '(b)', figure_font_name, axis_font_size, 0.52, -0.10);

trajectory_legend = add_bottom_legend(trajectory_3d_axis, ...
    [actual_xy_handle reference_xy_handle start_handle end_handle], ...
    {'UAV trajectory', 'Reference trajectory', 'Initial UAV position', 'Final UAV position'}, ...
    figure_font_name, legend_font_size, [0.18 0.035 0.64 0.07]);
set(trajectory_legend, 'NumColumns', 2);
export_figure(trajectory_figure, output_directory, 'fig1_trajectory_views', export_figures);

%% Figure 2: positions
position_figure = make_time_figure([12 13.5], [case_data.name ' positions']);
[position_x_axis, position_y_axis, position_h_axis, position_handles] = ...
    plot_three_reference_comparison(position_figure, t, ...
    {x, y, h}, {rx, ry, rh}, {'x [m]', 'y [m]', 'h [m]'}, ...
    time_plot_end, actual_color, reference_color, actual_line_width, ...
    reference_line_width, figure_font_name, axis_font_size, axes_line_width, grid_color);
arrange_three_axes_for_bottom_legend(position_x_axis, position_y_axis, position_h_axis);
add_bottom_legend(position_h_axis, position_handles, {'UAV', 'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(position_figure, output_directory, 'fig2_positions', export_figures);

%% Figure 3: velocities
velocity_figure = make_time_figure([12 13.5], [case_data.name ' velocities']);
[velocity_x_axis, velocity_y_axis, velocity_h_axis, velocity_handles] = ...
    plot_three_reference_comparison(velocity_figure, t, ...
    {xdot, ydot, hdot}, {rdx, rdy, rdh}, ...
    {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{h}$ [m/s]'}, ...
    time_plot_end, actual_color, reference_color, actual_line_width, ...
    reference_line_width, figure_font_name, axis_font_size, axes_line_width, grid_color);
set_latex_ylabels(velocity_x_axis, velocity_y_axis, velocity_h_axis, figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(velocity_x_axis, velocity_y_axis, velocity_h_axis);
add_bottom_legend(velocity_h_axis, velocity_handles, {'UAV', 'reference'}, ...
    figure_font_name, legend_font_size, bottom_legend_position);
export_figure(velocity_figure, output_directory, 'fig3_velocities', export_figures);

%% Figure 4: acceleration commands
acceleration_figure = make_time_figure([12 13.5], [case_data.name ' accelerations']);
[acc_x_axis, acc_y_axis, acc_h_axis, acc_handles] = ...
    plot_three_reference_comparison_with_limits(acceleration_figure, t, ...
    {xddot, yddot, hddot}, {rd2x, rd2y, rd2h}, a_con, ...
    {'$\ddot{x}$ [m/s$^2$]', '$\ddot{y}$ [m/s$^2$]', '$\ddot{h}$ [m/s$^2$]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
set_latex_ylabels(acc_x_axis, acc_y_axis, acc_h_axis, figure_font_name, axis_font_size);
arrange_three_axes_for_bottom_legend(acc_x_axis, acc_y_axis, acc_h_axis);
add_bottom_legend(acc_h_axis, acc_handles, {'UAV', 'reference', bounds_label}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(acceleration_figure, output_directory, 'fig4_accelerations', export_figures);

%% Figure 5: delta acceleration commands
delta_acceleration_figure = make_time_figure([12 13.5], [case_data.name ' delta accelerations']);
[dacc_x_axis, dacc_y_axis, dacc_h_axis, dacc_handles] = ...
    plot_three_reference_comparison_with_limits(delta_acceleration_figure, tk_u, ...
    {dU_d(:,1), dU_d(:,2), dU_d(:,3)}, ...
    {dU_ref(:,1), dU_ref(:,2), dU_ref(:,3)}, da_con, ...
    {'$\Delta\ddot{x}$ [m/s$^2$]', '$\Delta\ddot{y}$ [m/s$^2$]', '$\Delta\ddot{h}$ [m/s$^2$]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
set_latex_ylabels(dacc_x_axis, dacc_y_axis, dacc_h_axis, figure_font_name, axis_font_size);
set_delta_acceleration_axis(dacc_x_axis);
set_delta_acceleration_axis(dacc_y_axis);
set_delta_acceleration_axis(dacc_h_axis);
arrange_three_axes_for_bottom_legend(dacc_x_axis, dacc_y_axis, dacc_h_axis);
add_bottom_legend(dacc_h_axis, dacc_handles, {'UAV', 'reference', bounds_label}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(delta_acceleration_figure, output_directory, 'fig5_delta_accelerations', export_figures);

%% Figure 6: Vg, gamma, heading
state_figure = make_time_figure([12 13.5], [case_data.name ' states']);
[vg_axis, gamma_axis, psi_axis, state_handles] = ...
    plot_three_reference_comparison_with_limits(state_figure, t, ...
    {Vg, rad2deg(gamma), psi_deg}, {rVg, rad2deg(rGamma), rPsi_deg}, ...
    [C.Vg_min C.Vg_max; rad2deg(C.gamma_min) rad2deg(C.gamma_max); -Inf Inf], ...
    {'V_g [m/s]', '\gamma [deg]', '\psi [deg]'}, ...
    time_plot_end, actual_color, reference_color, limit_color, actual_line_width, ...
    reference_line_width, limit_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color);
yticks(vg_axis, sort(unique([yticks(vg_axis) 32])));
gamma_ylim = ylim(gamma_axis);
ylim(gamma_axis, [gamma_ylim(1) 20]);
gamma_ticks = yticks(gamma_axis);
gamma_ticks(gamma_ticks == 20) = [];
yticks(gamma_axis, gamma_ticks);
psi_ticks = yticks(psi_axis);
psi_ticks(psi_ticks == 200 | psi_ticks == -200) = [];
yticks(psi_axis, sort(unique([psi_ticks -180 180])));
arrange_three_axes_for_bottom_legend(vg_axis, gamma_axis, psi_axis);
add_bottom_legend(psi_axis, state_handles, {'UAV', 'reference', bounds_label}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(state_figure, output_directory, 'fig6_states', export_figures);

%% Figure 7: required UAV inputs
input_figure = make_time_figure([12 13.5], [case_data.name ' inputs']);
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
add_bottom_legend(phib_axis, input_handles, {'UAV', 'reference', bounds_label}, ...
    figure_font_name, legend_font_size, [0.30 0.035 0.40 0.05]);
export_figure(input_figure, output_directory, 'fig7_required_inputs', export_figures);

%% Figure 8: Euclidean distance
ed_figure = make_time_figure([12 7], [case_data.name ' Euclidean distance']);
ed_axis = axes('Parent', ed_figure, 'Position', [0.13 0.28 0.80 0.57]);
plot(ed_axis, t, metrics.ED, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
xlabel(ed_axis, 'Time [s]');
ylabel(ed_axis, 'Euclidean Distance [m]');
setup_time_axis(ed_axis, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
ed_xlabel = get(ed_axis, 'XLabel');
ed_xlabel.Position(2) = -0.30;
set_nonnegative_y_limits(ed_axis);
export_figure(ed_figure, output_directory, 'fig8_euclidean_distance', export_figures);

%% Figure 9: objective function value, only when present and meaningful
if isfield(run_data, 'J') && any(abs(run_data.J(:)) > 0)
    objective_figure = make_time_figure([12 7], [case_data.name ' objective']);
    objective_axis = axes('Parent', objective_figure, 'Position', [0.13 0.28 0.80 0.57]);
    plot(objective_axis, 1:numel(run_data.J), run_data.J, '-', 'Color', actual_color, ...
        'LineWidth', actual_line_width);
    xlabel(objective_axis, 'k');
    ylabel(objective_axis, 'J');
    setup_sample_axis(objective_axis, numel(run_data.J), figure_font_name, axis_font_size, axes_line_width, grid_color);
    objective_xlabel = get(objective_axis, 'XLabel');
    objective_xlabel.Position(2) = -0.30;
    set_nonnegative_y_limits(objective_axis);
    export_figure(objective_figure, output_directory, 'fig9_objective_value', export_figures);
end

%% Figure 10: total Lyapunov term
if strcmp(case_data.id, 'contractive_mpc')
    lyapunov_figure = make_time_figure([12 7], [case_data.name ' Lyapunov']);
    vtot_axis = axes('Parent', lyapunov_figure, 'Position', [0.13 0.28 0.80 0.57]);
    plot(vtot_axis, t, logs.Vtot, '-', 'Color', actual_color, 'LineWidth', actual_line_width);
    xlabel(vtot_axis, 'Time [s]');
    ylabel(vtot_axis, '$V(e_k)$', 'Interpreter', 'latex');
    setup_time_axis(vtot_axis, time_plot_end, figure_font_name, axis_font_size, ...
        axes_line_width, grid_color);
    vtot_xlabel = get(vtot_axis, 'XLabel');
    vtot_xlabel.Position(2) = -0.30;
    set_nonnegative_y_limits(vtot_axis);
    export_figure(lyapunov_figure, output_directory, 'fig10_lyapunov_terms', export_figures);
end

end

function [ref_acc_cont, ref_acc_samp] = get_reference_accelerations(t, tk)
ref_acc_cont = zeros(numel(t), 3);
for i = 1:numel(t)
    r = ref_state_hippodrome(t(i));
    ref_acc_cont(i,:) = r(7:9).';
end

ref_acc_samp = zeros(numel(tk)-1, 3);
for k = 1:numel(tk)-1
    r = ref_state_hippodrome(tk(k));
    ref_acc_samp(k,:) = r(7:9).';
end
end

function [ref_Th, ref_ng, ref_phib_deg] = get_reference_inputs(ref_acc_cont, ref_vel_cont, Vw, params)
N = size(ref_acc_cont, 1);
ref_phib = zeros(N,1);
ref_ng = zeros(N,1);
ref_Th = zeros(N,1);
for i = 1:N
    rdx = ref_vel_cont(i,1);
    rdy = ref_vel_cont(i,2);
    rdh = ref_vel_cont(i,3);
    rd2x = ref_acc_cont(i,1);
    rd2y = ref_acc_cont(i,2);
    rd2h = ref_acc_cont(i,3);

    rVg = sqrt(rdx^2 + rdy^2 + rdh^2);
    rGamma = asin(rdh / rVg);
    rPsi = atan2(rdy, rdx);
    [ref_phib(i), ref_ng(i), ref_Th(i), ~] = ...
        di_mapping(rd2x, rd2y, rd2h, rPsi, rGamma, rVg, Vw(i), params);
end
ref_phib_deg = rad2deg(ref_phib);
end

function [a_con, da_con] = get_acceleration_constraints(run_data, U_d)
if isfield(run_data, 'a_con')
    a_con = run_data.a_con;
elseif isfield(run_data, 'lb') && isfield(run_data, 'ub')
    a_con = [run_data.lb(1:3), run_data.ub(1:3)];
else
    a_min = min(U_d, [], 1).';
    a_max = max(U_d, [], 1).';
    a_con = [a_min, a_max];
end

if isfield(run_data, 'da_con')
    da_con = run_data.da_con;
    if isvector(da_con) && numel(da_con) == 2
        da_con = repmat(da_con(:).', 3, 1);
    end
else
    da_con = [-Inf Inf; -Inf Inf; -Inf Inf];
end
end

%% Plot helpers
function fig = make_time_figure(size_cm, figure_name)
fig = figure('Color', 'w', 'Name', figure_name);
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

function [ax1, ax2, ax3, ax4] = plot_four_single_signals(fig, time_vector, signals, labels, ...
    time_plot_end, actual_color, actual_line_width, figure_font_name, axis_font_size, ...
    axes_line_width, grid_color)
axes_positions = [
    0.13 0.785 0.80 0.13
    0.13 0.570 0.80 0.13
    0.13 0.355 0.80 0.13
    0.13 0.160 0.80 0.13
];
axes_list = gobjects(4,1);
for i = 1:4
    axes_list(i) = axes('Parent', fig, 'Position', axes_positions(i,:));
    plot(axes_list(i), time_vector, signals{i}, '-', 'Color', actual_color, ...
        'LineWidth', actual_line_width);
    ylabel(axes_list(i), labels{i});
    if i == 4
        xlabel(axes_list(i), 'Time [s]');
    end
    setup_time_axis(axes_list(i), time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color);
    add_subplot_label_at(axes_list(i), sprintf('(%c)', char('a' + i - 1)), ...
        figure_font_name, axis_font_size, 0.52, -0.22);
end
ax1 = axes_list(1);
ax2 = axes_list(2);
ax3 = axes_list(3);
ax4 = axes_list(4);
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

function setup_time_axis(ax, time_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color)
grid(ax, 'off');
xlim(ax, [0 time_plot_end]);
style_axis(ax, figure_font_name, axis_font_size, axes_line_width, grid_color);
pad_y_axis(ax, 0.12);
style_axis_labels(ax, figure_font_name, axis_font_size);
end

function setup_sample_axis(ax, sample_plot_end, figure_font_name, axis_font_size, axes_line_width, grid_color)
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
set(xlabel_handle, 'FontName', figure_font_name, 'FontSize', axis_font_size, 'FontWeight', 'bold');
set(ylabel_handle, 'FontName', figure_font_name, 'FontSize', axis_font_size, 'FontWeight', 'bold');
xlabel_handle.Units = 'normalized';
xlabel_handle.Position(2) = -0.44;
end

function set_latex_ylabels(ax1, ax2, ax3, figure_font_name, axis_font_size)
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

function legend_handle = add_bottom_legend(ax, handles, labels, figure_font_name, legend_font_size, legend_position)
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

function set_delta_acceleration_axis(ax)
ylim(ax, [-1.2 1.2]);
yticks(ax, [-1 -0.5 0 0.5 1]);
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

function export_figure(fig, output_directory, file_stem, export_figures)
if ~export_figures
    return;
end
print(fig, fullfile(output_directory, [file_stem '.svg']), '-dsvg', '-painters');
end
