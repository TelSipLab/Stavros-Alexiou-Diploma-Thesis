clc;

%% Controller metric table for thesis
script_directory = fileparts(mfilename('fullpath'));
output_directory = fullfile(script_directory, 'controller_metrics_table_thesis');
if ~exist(output_directory, 'dir')
    mkdir(output_directory);
end

controller_files = {
    'PID',            fullfile(script_directory, 'pid_ff_run.mat'),            '25 ms'
    'State Feedback', fullfile(script_directory, 'state_feedback_run.mat'),    '25 ms'
    'MPC',            fullfile(script_directory, 'contractive_mpc_run.mat'),   '100 ms'
};

row_names = {
    'Ts'
    'MSE_x'
    'MSE_y'
    'MSE_h'
    'MSE_total'
    'MAE_x'
    'MAE_y'
    'MAE_h'
    'MAE_total'
    'MED'
    'norm2_Ux'
    'norm2_Uy'
    'norm2_Uh'
    'Ju'
    'norm2_ng'
    'norm2_phib'
    'norm2_Th'
};

table_cells = cell(numel(row_names) + 1, size(controller_files, 1) + 1);
table_cells(1, :) = [{''}, controller_files(:,1).'];
table_cells(2:end, 1) = row_names;

for j = 1:size(controller_files, 1)
    if ~isfile(controller_files{j,2})
        error('Missing workspace for %s: %s', controller_files{j,1}, controller_files{j,2});
    end
    run_data = load(controller_files{j,2});
    metrics = run_data.metrics;
    for i = 1:numel(row_names)
        table_cells{i+1,j+1} = format_metric_value(row_names{i}, controller_files{j,3}, metrics);
    end
end

txt_file = fullfile(output_directory, 'controller_metrics_table_tab_delimited.txt');
csv_file = fullfile(output_directory, 'controller_metrics_table.csv');

writecell(table_cells, txt_file, 'Delimiter', 'tab');
writecell(table_cells, csv_file);

table_text = cell_table_to_tab_text(table_cells);
try
    clipboard('copy', table_text);
    fprintf('Table copied to clipboard.\n');
catch
    fprintf('Clipboard copy was not available in this MATLAB session.\n');
end

fprintf('\n%s\n', table_text);
fprintf('\nSaved table files in:\n%s\n', output_directory);

function value_text = format_metric_value(row_name, sample_time_text, metrics)
if strcmp(row_name, 'Ts')
    value_text = sample_time_text;
    return;
end

value = metrics.(row_name);
if strcmp(row_name, 'norm2_Th')
    value_text = sprintf('%.0f', value);
else
    value_text = sprintf('%.2f', value);
end
end

function table_text = cell_table_to_tab_text(table_cells)
lines = strings(size(table_cells, 1), 1);
for i = 1:size(table_cells, 1)
    row_values = strings(1, size(table_cells, 2));
    for j = 1:size(table_cells, 2)
        row_values(j) = string(table_cells{i,j});
    end
    lines(i) = strjoin(row_values, sprintf('\t'));
end
table_text = strjoin(lines, newline);
end
