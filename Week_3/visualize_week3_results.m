% visualize_week3_results.m
% ===========================================================================
% WEEK 3: Visualization script for direct collocation optimization results
% CORRECTED: Proper coordinate system - base at (x=0, y=1)
% ===========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3 RESULTS VISUALIZATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Load results ──────────────────────────────────────────────────────────
fprintf('[1/4] Loading results...\n');

load('week3_all_results.mat', 'results', 'week1_baseline', 'week2_baseline', 'p');

fprintf('  ✓ Results loaded from week3_all_results.mat\n\n');

simple_traj = results.simple.traj;
moderate_traj = results.moderate.traj;
hard_traj = results.hard.traj;

%% ── Figure 1: Comparison Bar Chart ───────────────────────────────────────
fprintf('[2/4] Generating comparison bar chart...\n');

figure('Position', [100, 100, 1200, 600]);
set(gcf, 'Color', 'w');

configs = {'Simple', 'Moderate', 'Hard'};
week1_data = [week1_baseline.simple.miss_cm, week1_baseline.moderate.miss_cm, week1_baseline.hard.miss_cm];
week2_data = [week2_baseline.simple.miss_cm, week2_baseline.moderate.miss_cm, week2_baseline.hard.miss_cm];
week3_data = [results.simple.miss_cm, results.moderate.miss_cm, results.hard.miss_cm];

bar_data = [week1_data; week2_data; week3_data]';

b = bar(bar_data);
b(1).FaceColor = [0.8, 0.3, 0.3];  % Week 1: Red
b(2).FaceColor = [0.3, 0.6, 0.8];  % Week 2: Blue
b(3).FaceColor = [0.2, 0.7, 0.3];  % Week 3: Green

set(gca, 'XTickLabel', configs);
ylabel('Miss Distance (cm)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Configuration', 'FontSize', 12, 'FontWeight', 'bold');
title('Week 1 vs Week 2 vs Week 3: Landing Accuracy Comparison', ...
    'FontSize', 14, 'FontWeight', 'bold');
legend({'Week 1 (Baseline)', 'Week 2 (Single-Shooting)', 'Week 3 (Direct Collocation)'}, ...
    'Location', 'northwest', 'FontSize', 11);
grid on;

hold on;
yline(0, 'k--', 'LineWidth', 2, 'Label', 'Perfect Landing', 'LabelHorizontalAlignment', 'left');

for i = 1:3
    if week2_data(i) > 0
        improvement = ((week2_data(i) - week3_data(i)) / week2_data(i)) * 100;
        text(i, week3_data(i) + 20, sprintf('↓%.0f%%', improvement), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.7, 0.3]);
    end
end

saveas(gcf, 'week3_comparison_bar.png');
fprintf('  ✓ Saved: week3_comparison_bar.png\n');

%% ── Figure 2: Trajectory Comparison (All Configs) ────────────────────────
fprintf('[3/4] Generating trajectory plots...\n');

figure('Position', [100, 100, 1600, 1000]);
set(gcf, 'Color', 'w');

configs_list = {'simple', 'moderate', 'hard'};
traj_list = {simple_traj, moderate_traj, hard_traj};
titles_list = {'Simple', 'Moderate', 'Hard'};

for c_idx = 1:3
    config_name = configs_list{c_idx};
    traj = traj_list{c_idx};
    
    obstacles = load_obstacle_config(config_name, p);
    p_temp = p;
    p_temp.obstacles = obstacles;
    
    % Subplot 1: Joint angles
    subplot(3, 4, (c_idx-1)*4 + 1);
    plot(traj.t, traj.q', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    title([titles_list{c_idx} ': Joint Angles']);
    legend({'q_1', 'q_2', 'q_3', 'q_4'}, 'Location', 'best');
    grid on;
    
    % Subplot 2: Joint velocities
    subplot(3, 4, (c_idx-1)*4 + 2);
    plot(traj.t, traj.qdot', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Velocities (rad/s)');
    title([titles_list{c_idx} ': Joint Velocities']);
    legend({'qdot_1', 'qdot_2', 'qdot_3', 'qdot_4'}, 'Location', 'best');
    grid on;
    
    % Subplot 3: Control torques
    subplot(3, 4, (c_idx-1)*4 + 3);
    plot(traj.t, traj.tau', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Torques (N·m)');
    title([titles_list{c_idx} ': Control Torques']);
    legend({'τ_1', 'τ_2', 'τ_3', 'τ_4'}, 'Location', 'best');
    grid on;
    
    % Subplot 4: Arm configuration at release - CORRECTED COORDINATES
    subplot(3, 4, (c_idx-1)*4 + 4);
    
    q_rel = traj.q(:, end);
    [joint_pos, ~] = forward_kinematics(q_rel, p);
    
    % CORRECTED: forward_kinematics returns [x; y] where:
    %   Row 1 (joint_pos(1,:)) = x coordinate (horizontal)
    %   Row 2 (joint_pos(2,:)) = y coordinate (vertical/height)
    % Base is at [0; p.y0] = [0; 1]
    % Plot with x on horizontal axis, y on vertical axis
    plot(joint_pos(1, :), joint_pos(2, :), 'b-o', 'LineWidth', 3, 'MarkerSize', 8);
    hold on;
    
    % Plot ground at y=0 (horizontal line)
    ground_x = linspace(-1, 3.5, 100);
    plot(ground_x, zeros(size(ground_x)), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground');
    
    % Plot circular obstacles - coordinates are (x, y)
    for obs_idx = 1:length(obstacles)
        obs = obstacles(obs_idx);
        theta = linspace(0, 2*pi, 100);
        circle_x = obs.cx + obs.r * cos(theta);
        circle_y = obs.cy + obs.r * sin(theta);
        fill(circle_x, circle_y, [0.8, 0.2, 0.2], 'FaceAlpha', 0.3);
        plot(circle_x, circle_y, 'r-', 'LineWidth', 2);
    end
    
    % Plot target (at ground y=0, horizontal position x=d)
    plot(p.task.d, 0, 'g*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Target');
    
    % Plot landing position (at y=0, x=x_land)
    plot(traj.x_land, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Landing');
    
    xlabel('x (m)', 'FontSize', 11);
    ylabel('y (m)', 'FontSize', 11);
    title([titles_list{c_idx} ': Arm at Release']);
    axis equal;
    grid on;
    legend({'Arm', 'Ground', 'Obstacles', 'Target', 'Landing'}, 'Location', 'best', 'FontSize', 9);
    
    % Set axis limits properly
    xlim([-1, 3.5]);
    ylim([-0.5, 2.5]);
end

sgtitle('Week 3: Direct Collocation Trajectories', 'FontSize', 16, 'FontWeight', 'bold');
saveas(gcf, 'week3_trajectories.png');
fprintf('  ✓ Saved: week3_trajectories.png\n');

%% ── Figure 3: Energy Analysis ─────────────────────────────────────────────
fprintf('[4/4] Generating energy analysis...\n');

figure('Position', [100, 100, 1000, 400]);
set(gcf, 'Color', 'w');

subplot(1, 2, 1);
energy_data = [results.simple.E_total, results.moderate.E_total, results.hard.E_total];
release_time_data = [results.simple.t_release, results.moderate.t_release, results.hard.t_release];

bar(energy_data);
set(gca, 'XTickLabel', configs);
ylabel('Total Energy (J)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Configuration', 'FontSize', 12, 'FontWeight', 'bold');
title('Energy Consumption', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

for i = 1:3
    text(i, energy_data(i) + max(energy_data)*0.05, sprintf('%.1f J', energy_data(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

subplot(1, 2, 2);
bar(release_time_data);
set(gca, 'XTickLabel', configs);
ylabel('Release Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Configuration', 'FontSize', 12, 'FontWeight', 'bold');
title('Optimized Release Time', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

for i = 1:3
    text(i, release_time_data(i) + 0.02, sprintf('%.2f s', release_time_data(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

sgtitle('Week 3: Energy and Timing Analysis', 'FontSize', 16, 'FontWeight', 'bold');
saveas(gcf, 'week3_energy_analysis.png');
fprintf('  ✓ Saved: week3_energy_analysis.png\n');

%% ── Summary Statistics ────────────────────────────────────────────────────
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  SUMMARY STATISTICS\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('Accuracy Improvement (Week 2 → Week 3):\n');
fprintf('  Simple:   %.2f cm → %.2f cm (%.1f%% improvement)\n', ...
    week2_data(1), week3_data(1), ((week2_data(1)-week3_data(1))/week2_data(1))*100);
fprintf('  Moderate: %.2f cm → %.2f cm (%.1f%% improvement)\n', ...
    week2_data(2), week3_data(2), ((week2_data(2)-week3_data(2))/week2_data(2))*100);
fprintf('  Hard:     %.2f cm → %.2f cm (%.1f%% improvement)\n', ...
    week2_data(3), week3_data(3), ((week2_data(3)-week3_data(3))/week2_data(3))*100);

fprintf('\nOptimization Performance:\n');
fprintf('  Average solve time: %.1f s\n', mean([results.simple.time_s, results.moderate.time_s, results.hard.time_s]));
fprintf('  Average iterations: %.0f\n', mean([results.simple.iterations, results.moderate.iterations, results.hard.iterations]));

fprintf('\nPhysical Metrics:\n');
fprintf('  Average energy: %.1f J\n', mean(energy_data));
fprintf('  Average release time: %.2f s\n', mean(release_time_data));

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  VISUALIZATION COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');
fprintf('Generated files:\n');
fprintf('  - week3_comparison_bar.png\n');
fprintf('  - week3_trajectories.png\n');
fprintf('  - week3_energy_analysis.png\n');
fprintf('\n');