% visualize_complete_trajectory.m
% ===========================================================================
% This script shows the full animation including the manipulation and ballistic phase.... 
% ===========================================================================

function visualize_complete_trajectory(config_name)

if nargin < 1
    config_name = 'hard';
end

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  COMPLETE TRAJECTORY VISUALIZATION: %s\n', upper(config_name));
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Load data ─────────────────────────────────────────────────────────────
addpath('/mnt/project');

load('week3_all_results.mat', 'results', 'p');
traj = results.(config_name).traj;

obstacles = load_obstacle_config(config_name, p);
p.obstacles = obstacles;

%% ── Compute ballistic trajectory after release ───────────────────────────
q_rel = traj.q(:, end);
qdot_rel = traj.qdot(:, end);

% Call ballistic_trajectory with empty t_span so it auto-computes until landing
flight = ballistic_trajectory(q_rel, qdot_rel, p, []);

fprintf('  Release time:    %.2f s\n', traj.t_release);
fprintf('  Flight time:     %.2f s\n', flight.t_land);
fprintf('  Total time:      %.2f s\n', traj.t_release + flight.t_land);
fprintf('  Landing x:       %.3f m\n', flight.x_land);
fprintf('  Miss distance:   %.2f cm\n', abs(flight.x_land - p.task.d)*100);
fprintf('\n');

%% ── Create 4x4 visualization ─────────────────────────────────────────────
fprintf('  Creating visualization...\n');

n_manip_frames = 8;
n_flight_frames = 8;

manip_times = linspace(0, traj.t_release, n_manip_frames);
flight_times = linspace(0, flight.t_land, n_flight_frames);

fig = figure('Position', [50, 50, 1800, 1000]);
set(gcf, 'Color', 'w');

frame_count = 0;

%% ── Manipulation phase frames ────────────────────────────────────────────
for i = 1:n_manip_frames
    frame_count = frame_count + 1;
    subplot(4, 4, frame_count);
    hold on;
    
    t_now = manip_times(i);
    
    % Interpolate state at this time
    q_now = interp1(traj.t, traj.q', t_now)';
    
    % Get joint positions
    [joint_pos, ~] = forward_kinematics(q_now, p);
    
    % --- Draw environment ---
    % Ground
    plot([-1, 3.5], [0, 0], 'k-', 'LineWidth', 2);
    
    % Pedestal
    plot([0, 0], [0, p.y0], '-', 'Color', [0.3, 0.3, 0.3], 'LineWidth', 3);
    
    % Obstacles
    draw_obstacles(obstacles);
    
    % Target
    plot(p.task.d, 0, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.1, 0.8, 0.1], ...
        'MarkerEdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
    
    % Full ballistic arc (preview, thin dashed)
    plot(flight.pos(1, :), flight.pos(2, :), '--', 'Color', [0.6, 0.6, 0.6], 'LineWidth', 1);
    
    % --- Draw arm ---
    plot(joint_pos(1, :), joint_pos(2, :), 'b-', 'LineWidth', 3);
    plot(joint_pos(1, :), joint_pos(2, :), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
    
    % --- Draw object attached to end-effector ---
    alpha_N = sum(q_now);
    R_N = [cos(alpha_N), -sin(alpha_N); sin(alpha_N), cos(alpha_N)];
    obj_pos = joint_pos(:, end) + R_N * p.obj.r_gc;
    draw_object_circle(obj_pos, p.obj.r, [0.2, 0.8, 0.2]);
    
    % Formatting
    xlabel('x (m)', 'FontSize', 8);
    ylabel('y (m)', 'FontSize', 8);
    title(sprintf('MANIP t=%.2fs', t_now), 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0, 0, 0.7]);
    axis equal; grid on;
    xlim([-0.8, 3.5]); ylim([-0.3, 2.5]);
end

%% ── Flight phase frames ─────────────────────────────────────────────────
for i = 1:n_flight_frames
    frame_count = frame_count + 1;
    subplot(4, 4, frame_count);
    hold on;
    
    t_fl = flight_times(i);
    t_total = traj.t_release + t_fl;
    
    % --- Draw environment ---
    % Ground
    plot([-1, 3.5], [0, 0], 'k-', 'LineWidth', 2);
    
    % Pedestal
    plot([0, 0], [0, p.y0], '-', 'Color', [0.3, 0.3, 0.3], 'LineWidth', 3);
    
    % Obstacles
    draw_obstacles(obstacles);
    
    % Target
    plot(p.task.d, 0, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.1, 0.8, 0.1], ...
        'MarkerEdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
    
    % --- Draw arm frozen at release (faded) ---
    [joint_pos_rel, ~] = forward_kinematics(q_rel, p);
    plot(joint_pos_rel(1, :), joint_pos_rel(2, :), '-', 'Color', [0.7, 0.7, 1], 'LineWidth', 2);
    plot(joint_pos_rel(1, :), joint_pos_rel(2, :), 'o', 'Color', [0.7, 0.7, 1], ...
        'MarkerSize', 5, 'MarkerFaceColor', [0.7, 0.7, 1]);
    
    % --- Draw full ballistic arc (thin dashed green) ---
    plot(flight.pos(1, :), flight.pos(2, :), 'g--', 'LineWidth', 2);
    
    % --- Compute object position analytically (same as Week 1 line 236) ---
    obj_pos_now = flight.r0 + flight.v0 * t_fl + 0.5 * [0; -p.g] * t_fl^2;
    
    % --- Draw trajectory traveled so far (thick solid green) ---
    idx_now = find(flight.t >= t_fl, 1);
    if isempty(idx_now)
        idx_now = length(flight.t);
    end
    if idx_now > 1
        plot(flight.pos(1, 1:idx_now), flight.pos(2, 1:idx_now), 'g-', 'LineWidth', 3.5);
    end
    
    % --- Draw object at current position (orange) ---
    draw_object_circle(obj_pos_now, p.obj.r, [1.0, 0.5, 0.0]);
    
    % --- Draw landing marker ---
    if flight.landed
        plot(flight.x_land, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 3);
    end
    
    % Formatting
    xlabel('x (m)', 'FontSize', 8);
    ylabel('y (m)', 'FontSize', 8);
    title(sprintf('FLIGHT t=%.2fs', t_total), 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8, 0.4, 0]);
    axis equal; grid on;
    xlim([-0.8, 3.5]); ylim([-0.3, 2.5]);
end

sgtitle(sprintf('Complete Trajectory: %s Configuration (Manipulation + Flight)\nMiss: %.2f cm', ...
    upper(config_name), abs(flight.x_land - p.task.d)*100), ...
    'FontSize', 14, 'FontWeight', 'bold');

% Save
saveas(gcf, sprintf('complete_trajectory_%s.png', config_name));
fprintf('  ✓ Saved: complete_trajectory_%s.png\n', config_name);

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  VISUALIZATION COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Blue titles   = Manipulation phase (arm moving)\n');
fprintf('  Orange titles  = Flight phase (ballistic motion)\n');
fprintf('  Green dashed   = Full ballistic trajectory arc\n');
fprintf('  Green solid    = Trajectory traveled so far\n');
fprintf('  Orange circle  = Object current position\n');
fprintf('  Red X          = Landing position\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

end

%% ═══════════════════════════════════════════════════════════════════════════
%% HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════════════════════

function draw_obstacles(obstacles)
% Draw circular obstacles
    for obs_idx = 1:length(obstacles)
        obs = obstacles(obs_idx);
        theta = linspace(0, 2*pi, 100);
        cx = obs.cx + obs.r * cos(theta);
        cy = obs.cy + obs.r * sin(theta);
        fill(cx, cy, [0.8, 0.2, 0.2], 'FaceAlpha', 0.4, 'EdgeColor', 'r', 'LineWidth', 2);
    end
end

function draw_object_circle(pos, r, color)
% Draw object as a filled circle
    theta = linspace(0, 2*pi, 50);
    cx = pos(1) + r * cos(theta);
    cy = pos(2) + r * sin(theta);
    fill(cx, cy, color, 'FaceAlpha', 0.8, 'EdgeColor', color * 0.6, 'LineWidth', 2);
end