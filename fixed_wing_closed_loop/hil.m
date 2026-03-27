% read csv file
times_ms = readmatrix('computational_times.csv');

% if the values are in seconds instead of milliseconds, use:
% times_ms = 1000*readmatrix('computational_times.csv');

% statistics
t_min  = min(times_ms);
t_max  = max(times_ms);
t_mean = mean(times_ms);

% print results
fprintf('Minimum time: %.6f ms\n', t_min);
fprintf('Maximum time: %.6f ms\n', t_max);
fprintf('Mean time:    %.6f ms\n', t_mean);

% step index
k = 1:length(times_ms);

% plot
figure('Color','w','Position',[100 100 900 420]);
plot(k, times_ms, 'LineWidth', 1.4);
hold on;
yline(t_mean, '--', 'Mean', 'LineWidth', 1.2);
grid on;

xlabel('k', 'FontSize', 12);
ylabel('Computational time [ms]', 'FontSize', 12);
title('Embedded HIL Computational Time', 'FontSize', 13);

set(gca, 'FontSize', 11, 'LineWidth', 1);

% save as EPS
print(gcf, 'HIL_time_plot', '-depsc');