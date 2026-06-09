% visualize_collision_check.m
% ===========================================================================
% This script checks for the collisions during manipulation phase.... 
% ===========================================================================

function visualize_collision_check(config_name)

if nargin < 1
    config_name = 'hard';
end

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  VISUAL COLLISION VERIFICATION: %s\n', upper(config_name));
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Load data ─────────────────────────────────────────────────────────────
addpath('/mnt/project');

load('week3_all_results.mat', 'results', 'p');
traj = results.(config_name).traj;

obstacles = load_obstacle_config(config_name, p);
p.obstacles = obstacles;

fprintf('  Trajectory time: %.2f s\n', traj.t_release);
fprintf('  Time points: %d\n', length(traj.t));
fprintf('  Obstacles: %d\n', length(obstacles));
fprintf('\n');

%% ── Create multi-frame visualization ──────────────────────────────────────
fprintf('  Creating visualization...\n');

% Select frames to show (spread evenly through trajectory)
n_frames_to_show = min(12, length(traj.t));
frame_indices = round(linspace(1, length(traj.t), n_frames_to_show));

% Create figure with subplots
fig = figure('Position', [50, 50, 1600, 1200]);
set(gcf, 'Color', 'w');

% Calculate subplot layout
n_cols = 4;
n_rows = ceil(n_frames_to_show / n_cols);

for i = 1:n_frames_to_show
    subplot(n_rows, n_cols, i);
    
    frame_idx = frame_indices(i);
    t_now = traj.t(frame_idx);
    q_now = traj.q(:, frame_idx);
    
    % Get joint positions
    [joint_pos, ~] = forward_kinematics(q_now, p);
    
    % Plot arm
    plot(joint_pos(1, :), joint_pos(2, :), 'b-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    hold on;
    
    % Plot ground
    plot([-1, 3.5], [0, 0], 'k-', 'LineWidth', 2);
    
    % Plot obstacles with safety margin visualization
    for obs_idx = 1:length(obstacles)
        obs = obstacles(obs_idx);
        theta = linspace(0, 2*pi, 100);
        
        % Obstacle itself
        circle_x = obs.cx + obs.r * cos(theta);
        circle_y = obs.cy + obs.r * sin(theta);
        fill(circle_x, circle_y, [0.8, 0.2, 0.2], 'FaceAlpha', 0.4, 'EdgeColor', 'r', 'LineWidth', 2);
        
        % Safety margin (10cm) - to visualize clearance
        margin_r = obs.r + 0.1;
        margin_x = obs.cx + margin_r * cos(theta);
        margin_y = obs.cy + margin_r * sin(theta);
        plot(margin_x, margin_y, 'r--', 'LineWidth', 1);
    end
    
    % Plot object at end-effector
    ee_pos = joint_pos(:, end);
    alpha_N = sum(q_now);
    R_N = [cos(alpha_N), -sin(alpha_N);
           sin(alpha_N),  cos(alpha_N)];
    obj_pos = ee_pos + R_N * p.obj.r_gc;
    
    % Object as circle with radius
    obj_theta = linspace(0, 2*pi, 50);
    obj_circle_x = obj_pos(1) + p.obj.r * cos(obj_theta);
    obj_circle_y = obj_pos(2) + p.obj.r * sin(obj_theta);
    fill(obj_circle_x, obj_circle_y, [0.2, 0.8, 0.2], 'FaceAlpha', 0.6, 'EdgeColor', 'g', 'LineWidth', 2);
    
    % Check and mark any collisions at this frame
    has_collision = false;
    
    % Check arm links
    for link_i = 1:p.N
        p1 = joint_pos(:, link_i);
        p2 = joint_pos(:, link_i + 1);
        
        query.type = 'link';
        query.p1 = p1;
        query.p2 = p2;
        
        [is_collision, ~] = check_collision(query, obstacles, p);
        if is_collision
            has_collision = true;
            % Highlight colliding link
            plot([p1(1), p2(1)], [p1(2), p2(2)], 'r-', 'LineWidth', 5);
        end
    end
    
    % Check object
    query.type = 'object';
    query.pos = obj_pos;
    query.theta = alpha_N;
    
    [is_collision, ~] = check_collision(query, obstacles, p);
    if is_collision
        has_collision = true;
        % Highlight colliding object
        plot(obj_circle_x, obj_circle_y, 'r-', 'LineWidth', 3);
    end
    
    % Plot target
    plot(p.task.d, 0, 'g*', 'MarkerSize', 12, 'LineWidth', 2);
    
    % Formatting
    xlabel('x (m)', 'FontSize', 9);
    ylabel('y (m)', 'FontSize', 9);
    title(sprintf('t = %.2f s%s', t_now, ternary(has_collision, ' ⚠ COLLISION', '')), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', ternary(has_collision, [1, 0, 0], [0, 0, 0]));
    axis equal;
    grid on;
    xlim([-0.5, 3.5]);
    ylim([-0.3, 2.2]);
    
    % Add collision indicator
    if has_collision
        text(0.5, 2.0, '⚠ COLLISION', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    end
end

sgtitle(sprintf('Visual Collision Check: %s Configuration', upper(config_name)), ...
    'FontSize', 16, 'FontWeight', 'bold');

% Save figure
saveas(gcf, sprintf('collision_check_%s.png', config_name));
fprintf('  ✓ Saved: collision_check_%s.png\n', config_name);

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  VISUALIZATION COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Visually inspect frames for red highlights indicating collisions.\n');
fprintf('  Red dashed circles show 10cm safety margin around obstacles.\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

end

% Helper function for ternary operator
function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end