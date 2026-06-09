clear all;
close all;
clc;

%% Load all results
load('week2_all_results.mat');

%% Extract data from results struct
configs = fieldnames(results);
n_configs = length(configs);

% Pre-allocate arrays
x_land = zeros(n_configs, 1);
miss_cm = zeros(n_configs, 1);
E_total = zeros(n_configs, 1);
t_release_opt = zeros(n_configs, 1);
iterations = zeros(n_configs, 1);
time_s = zeros(n_configs, 1);

%% Fill arrays
for i = 1:n_configs
    config = configs{i};

    x_land(i)       = results.(config).x_land;
    miss_cm(i)      = results.(config).miss_cm;
    E_total(i)      = results.(config).E_total;
    t_release_opt(i)= results.(config).t_release;
    iterations(i)   = results.(config).iterations;
    time_s(i)       = results.(config).time_s;
end

%% Week 1 baseline data
week1_miss = [117.79; 1000; 1000];
week1_labels = {'SIMPLE', 'MODERATE', 'HARD'};

%% Create comprehensive comparison figure
figure('Position', [100 100 1400 900]);

x = 1:n_configs;

%% ========================================================================
% Plot 1: Landing Accuracy Comparison
%% ========================================================================
subplot(2,3,1);

bar(x - 0.2, week1_miss, 0.4, ...
    'FaceColor', 'b', ...
    'DisplayName', 'Week 1 Baseline');

hold on;

bar(x + 0.2, miss_cm, 0.4, ...
    'FaceColor', 'r', ...
    'DisplayName', 'Week 2 Optimized');

% Horizontal target reference line
xl = xlim;
plot(xl, [297.7 297.7], 'k--', ...
    'LineWidth', 2, ...
    'DisplayName', 'Target (2.977 m)');

xlabel('Configuration');
ylabel('Miss Distance [cm]');
title('Landing Accuracy: Week 1 vs Week 2');

set(gca, 'XTick', x);
set(gca, 'XTickLabel', week1_labels);

legend('Location', 'best');
grid on;

%% ========================================================================
% Plot 2: Improvement
%% ========================================================================
subplot(2,3,2);

improvement = week1_miss - miss_cm;

% Create bar plot
b = bar(x, improvement, 0.6, 'FaceColor', 'flat');

% Set bar colors individually
for i = 1:n_configs
    if improvement(i) >= 0
        b.CData(i,:) = [0.2 0.8 0.2];   % Green
    else
        b.CData(i,:) = [0.8 0.2 0.2];   % Red
    end
end

hold on;

% Zero reference line
xl = xlim;
plot(xl, [0 0], 'k-', 'LineWidth', 1);

ylabel('Improvement [cm]');
title('Miss Distance Improvement');

set(gca, 'XTick', x);
set(gca, 'XTickLabel', week1_labels);

grid on;

% Add value labels
for i = 1:n_configs

    if improvement(i) >= 0
        y_pos = improvement(i) + 5;
    else
        y_pos = improvement(i) - 5;
    end

    text(x(i), y_pos, sprintf('%.1f', improvement(i)), ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold');
end

%% ========================================================================
% Plot 3: Energy Consumption
%% ========================================================================
subplot(2,3,3);

bar(x, E_total, 0.6, ...
    'FaceColor', [0.2 0.5 0.8]);

ylabel('Energy [J]');
title('Energy Consumed During Optimization');

set(gca, 'XTick', x);
set(gca, 'XTickLabel', week1_labels);

grid on;

% Add value labels
for i = 1:n_configs

    text(x(i), ...
        E_total(i) + max(E_total)*0.02, ...
        sprintf('%.1f J', E_total(i)), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 9);
end

%% ========================================================================
% Plot 4: Optimized Release Time
%% ========================================================================
subplot(2,3,4);

bar(x - 0.2, ones(n_configs,1)*1.2, 0.4, ...
    'FaceColor', 'b', ...
    'DisplayName', 'Week 1 (fixed)');

hold on;

bar(x + 0.2, t_release_opt, 0.4, ...
    'FaceColor', 'r', ...
    'DisplayName', 'Week 2 (optimized)');

ylabel('Release Time [s]');
title('Release Time: Fixed vs Optimized');

set(gca, 'XTick', x);
set(gca, 'XTickLabel', week1_labels);

ylim([0.5 2.0]);

legend('Location', 'best');
grid on;

% Add value labels
for i = 1:n_configs

    text(x(i)+0.2, ...
        t_release_opt(i)+0.05, ...
        sprintf('%.2f s', t_release_opt(i)), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 9);
end

%% ========================================================================
% Plot 5: Convergence Metrics
%% ========================================================================
subplot(2,3,5);

% LEFT AXIS
yyaxis left

bar(x - 0.2, iterations, 0.4, ...
    'FaceColor', [0.2 0.8 0.8], ...
    'DisplayName', 'Iterations');

ylabel('Iterations');

ylim([0 max(iterations)*1.2]);

% RIGHT AXIS
yyaxis right

plot(x, time_s, 'ro-', ...
    'LineWidth', 2, ...
    'MarkerSize', 8, ...
    'DisplayName', 'Time');

ylabel('Time [s]');

title('Optimization Convergence');

set(gca, 'XTick', x);
set(gca, 'XTickLabel', week1_labels);

grid on;

%% ========================================================================
% Plot 6: Summary Table
%% ========================================================================
subplot(2,3,6);

axis off;

% Create summary text
summary_str = sprintf([ ...
    'WEEK 2 OPTIMIZATION SUMMARY\n' ...
    '================================\n\n']);

for i = 1:n_configs

    config = week1_labels{i};

    summary_str = sprintf('%s%s Config:\n', ...
        summary_str, config);

    summary_str = sprintf('%s  Landing x:     %.3f m\n', ...
        summary_str, x_land(i));

    summary_str = sprintf('%s  Miss distance: %.2f cm\n', ...
        summary_str, miss_cm(i));

    summary_str = sprintf('%s  Week1 miss:    %.2f cm\n', ...
        summary_str, week1_miss(i));

    summary_str = sprintf('%s  Improvement:   %+ .2f cm\n', ...
        summary_str, improvement(i));

    summary_str = sprintf('%s  Release time:  %.2f s\n', ...
        summary_str, t_release_opt(i));

    summary_str = sprintf('%s  Energy:        %.2f J\n', ...
        summary_str, E_total(i));

    summary_str = sprintf('%s  Iterations:    %d\n', ...
        summary_str, iterations(i));

    summary_str = sprintf('%s  Time:          %.1f s\n\n', ...
        summary_str, time_s(i));
end

text(0.05, 1.2, summary_str, ...
    'FontName', 'Courier', ...
    'FontSize', 7.5, ...
    'VerticalAlignment', 'top', ...
    'HorizontalAlignment', 'left');

%% ========================================================================
% Overall Figure Title
%% ========================================================================
annotation('textbox', [0 0.95 1 0.05], ...
    'String', 'Week 2 Trajectory Optimization: Complete Results Summary', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

%% Save figure
savefig('week2_results_summary.fig');
print('week2_results_summary.png', '-dpng', '-r150');