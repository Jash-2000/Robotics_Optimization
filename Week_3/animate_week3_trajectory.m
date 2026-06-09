% animate_week3_trajectory.m
% ===========================================================================
% Animates the optimized Week 3 trajectory for a specific configuration
% CORRECTED: Proper x,y convention - base at (x=0, y=1)
% ===========================================================================

function animate_week3_trajectory(config_name)

if nargin < 1
    config_name = 'simple';
end

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3 TRAJECTORY ANIMATION: %s\n', upper(config_name));
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Load data ─────────────────────────────────────────────────────────────
fprintf('[1/3] Loading trajectory...\n');

load('week3_all_results.mat', 'results', 'p');

traj = results.(config_name).traj;

fprintf('  ✓ Loaded from week3_all_results.mat\n');
fprintf('  Release time: %.2f s\n', traj.t_release);
fprintf('  Landing x: %.3f m (miss: %.2f cm)\n', traj.x_land, abs(traj.x_land - p.task.d)*100);
fprintf('\n');

obstacles = load_obstacle_config(config_name, p);
p.obstacles = obstacles;

%% ── Setup figure ──────────────────────────────────────────────────────────
fprintf('[2/3] Setting up animation...\n');

fig = figure('Position', [100, 100, 1400, 800]);
set(gcf, 'Color', 'w');
set(gcf, 'Renderer', 'painters');  % Use painters renderer for consistent frames

% Axis limits
x_min = -1;
x_max = 3.5;
y_min = -0.5;
y_max = 2.5;

%% ── Animate trajectory ────────────────────────────────────────────────────
fprintf('[3/3] Animating...\n');

fps = 30;
dt_anim = 1/fps;
t_total = traj.t_release + 1.0;
n_frames = ceil(t_total / dt_anim);

% Interpolate trajectory for smooth animation
n_manip_frames = round(traj.t_release * fps);
if n_manip_frames < 1
    n_manip_frames = 1;
end
t_manip = linspace(0, traj.t_release, n_manip_frames);
q_interp = interp1(traj.t, traj.q', t_manip)';
qdot_interp = interp1(traj.t, traj.qdot', t_manip)';

video_filename = sprintf('week3_animation_%s.mp4', config_name);
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
open(v);

for frame = 1:n_frames
    clf;
    
    t_now = frame * dt_anim;
    
    %% Left panel: Arm visualization
    subplot(1, 2, 1);
    hold on;
    
    % Plot ground at y=0
    plot([x_min, x_max], [0, 0], 'k-', 'LineWidth', 2);
    
    % Plot circular obstacles
    for obs_idx = 1:length(p.obstacles)
        obs = p.obstacles(obs_idx);
        theta = linspace(0, 2*pi, 100);
        circle_x = obs.cx + obs.r * cos(theta);
        circle_y = obs.cy + obs.r * sin(theta);
        fill(circle_x, circle_y, [0.8, 0.2, 0.2], 'FaceAlpha', 0.3);
        plot(circle_x, circle_y, 'r-', 'LineWidth', 2);
    end
    
    % Plot target (at y=0, x=d)
    plot(p.task.d, 0, 'g*', 'MarkerSize', 20, 'LineWidth', 3);
    text(p.task.d, -0.15, 'Target', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    
    % Determine state based on time
    if t_now <= traj.t_release
        % Manipulation phase
        frame_idx = max(1, min(size(q_interp, 2), round((t_now / traj.t_release) * size(q_interp, 2))));
        q_now = q_interp(:, frame_idx);
        
        [joint_pos, ~] = forward_kinematics(q_now, p);
        
        % Plot arm: joint_pos is [x; y]
        plot(joint_pos(1, :), joint_pos(2, :), 'b-o', 'LineWidth', 4, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        
        % Plot object at end-effector
        obj_pos = joint_pos(:, end) + [p.obj.r_gc(1); p.obj.r_gc(2)];
        plot(obj_pos(1), obj_pos(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        
    else
        % Flight phase
        q_rel = traj.q(:, end);
        qdot_rel = traj.qdot(:, end);
        
        [joint_pos_rel, ~] = forward_kinematics(q_rel, p);
        plot(joint_pos_rel(1, :), joint_pos_rel(2, :), 'b-o', 'LineWidth', 2, 'MarkerSize', 8, ...
            'MarkerFaceColor', 'b', 'Color', [0.5, 0.5, 1, 0.5]);
        
        % Ballistic trajectory
        t_flight_total = t_now - traj.t_release;
        t_flight = linspace(0, t_flight_total, 50);
        flight_traj = ballistic_trajectory(q_rel, qdot_rel, p, t_flight);
        
        plot(flight_traj.pos(1, :), flight_traj.pos(2, :), 'g--', 'LineWidth', 2);
        
        if t_flight_total > 0 && size(flight_traj.pos, 2) > 0
            plot(flight_traj.pos(1, end), flight_traj.pos(2, end), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        end
    end
    
    xlabel('x (m)', 'FontSize', 12);
    ylabel('y (m)', 'FontSize', 12);
    title(sprintf('Configuration: %s | t = %.2f s', upper(config_name), t_now), ...
        'FontSize', 14, 'FontWeight', 'bold');
    axis([x_min, x_max, y_min, y_max]);
    axis equal;
    grid on;
    
    %% Right panel: State plots
    if t_now <= traj.t_release
        frame_idx_plot = max(1, min(size(q_interp, 2), round((t_now / traj.t_release) * size(q_interp, 2))));
        
        subplot(2, 2, 2);
        plot(t_manip(1:frame_idx_plot), q_interp(:, 1:frame_idx_plot)', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint Angles (rad)');
        title('Joint Angles');
        legend({'q_1', 'q_2', 'q_3', 'q_4'}, 'Location', 'best');
        grid on;
        xlim([0, traj.t_release]);
        
        subplot(2, 2, 4);
        plot(t_manip(1:frame_idx_plot), qdot_interp(:, 1:frame_idx_plot)', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint Velocities (rad/s)');
        title('Joint Velocities');
        legend({'qdot_1', 'qdot_2', 'qdot_3', 'qdot_4'}, 'Location', 'best');
        grid on;
        xlim([0, traj.t_release]);
    else
        subplot(2, 2, 2);
        plot(t_manip, q_interp', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint Angles (rad)');
        title('Joint Angles (Full)');
        legend({'q_1', 'q_2', 'q_3', 'q_4'}, 'Location', 'best');
        grid on;
        xlim([0, traj.t_release]);
        
        subplot(2, 2, 4);
        plot(t_manip, qdot_interp', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('Joint Velocities (rad/s)');
        title('Joint Velocities (Full)');
        legend({'qdot_1', 'qdot_2', 'qdot_3', 'qdot_4'}, 'Location', 'best');
        grid on;
        xlim([0, traj.t_release]);
    end
    
    % Capture frame with consistent size
    drawnow;  % Ensure all graphics are rendered
    frame_data = getframe(fig);
    
    % On first frame, check size and adjust video if needed
    if frame == 1
        fprintf('  Video frame size: %d × %d\n', size(frame_data.cdata, 2), size(frame_data.cdata, 1));
    end
    
    writeVideo(v, frame_data);
    
    if mod(frame, 10) == 0
        fprintf('  Frame %d/%d (%.1f%%)\n', frame, n_frames, 100*frame/n_frames);
    end
end

close(v);

fprintf('  ✓ Animation saved: %s\n', video_filename);
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  ANIMATION COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

end