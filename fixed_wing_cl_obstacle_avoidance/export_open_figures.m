function export_open_figures()
% Export all currently open figures as PNG files.
%
% Usage:
%   export_open_figures

output_dir = 'diagnostic_figures';
mkdir(output_dir);

figs = flipud(findall(0, 'Type', 'figure'));

for i = 1:numel(figs)
    filename = fullfile(output_dir, sprintf('figure_%02d.png', i));
    exportgraphics(figs(i), filename, 'Resolution', 300);
end

fprintf('Exported %d open figures to: %s\n', numel(figs), output_dir);

end
