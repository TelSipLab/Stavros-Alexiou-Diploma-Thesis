function export_saved_run_plots(workspace_file)
% Load a saved run, generate the standard plots, and export them as PNG.
%
% Usage:
%   export_saved_run_plots('saved_run.mat')

load(workspace_file);

output_dir = 'diagnostic_figures';
mkdir(output_dir);

a_con = [lb(1:3) ub(1:3)];

close all;
plots_func(t_total, logs, metrics, J, exitflag, obstacles, obst_params, a_con, da_con);

figs = flipud(findall(0, 'Type', 'figure'));

for i = 1:numel(figs)
    filename = fullfile(output_dir, sprintf('figure_%02d.png', i));
    exportgraphics(figs(i), filename, 'Resolution', 300);
end

fprintf('Exported %d figures to: %s\n', numel(figs), output_dir);

end
