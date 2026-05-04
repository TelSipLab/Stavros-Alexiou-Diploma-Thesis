clc; close all;

[file_name, file_path] = uigetfile('*.mat', 'Select saved simulation workspace');
if isequal(file_name, 0)
    disp('No saved workspace selected.');
    return;
end

workspace_file = fullfile(file_path, file_name);
loaded_data = load(workspace_file);

required_vars = {'t_total', 'logs', 'metrics', 'J', 'exitflag', 'obstacles', 'obst_params'};
missing_vars = required_vars(~isfield(loaded_data, required_vars));
if ~isempty(missing_vars)
    error('The selected workspace is missing required variables: %s', ...
        strjoin(missing_vars, ', '));
end

fprintf('Loaded saved workspace:\n%s\n', workspace_file);

plots_func(loaded_data.t_total, loaded_data.logs, loaded_data.metrics, ...
    loaded_data.J, loaded_data.exitflag, loaded_data.obstacles, loaded_data.obst_params);

