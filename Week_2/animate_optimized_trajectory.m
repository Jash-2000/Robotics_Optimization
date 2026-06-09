function animate_optimized_trajectory(z, p, config_name)
% animate_optimized_trajectory.m
% =========================================================================
% Animates the trajectory resulting from optimized decision variables z.
% Similar to animate_simulation.m from Week 1, but uses optimized torques.
%
% INPUTS:
%   z           – [4M×1] optimized decision variables
%   p           – parameter struct from params.m (includes p.obstacles)
%   config_name – string: 'simple', 'moderate', 'hard' (for title)
%
% OUTPUT:
%   Opens figure window with animation of optimized trajectory
%
% USAGE:
%   animate_optimized_trajectory(z_opt, p, 'simple');
% =========================================================================

%% ── Simulate trajectory with optimized torques ────────────────────────────
[tau_func, t_release] = unpack_torques(z, p);

q0 = p.q0;
qdot0 = p.qdot0;
y0 = [q0; qdot0];

% Simulation time span - use optimized release time
dt_out = p.sim.dt;
tspan = 0:dt_out:t_release;

ode_opts = p.sim.ode_opts;
ode_func = @(t, y) arm_ode(t, y, tau_func, p);

% Integrate
[t_vec, y_vec] = ode45(ode_func, tspan, y0, ode_opts);

% Extract release state
q_rel = y_vec(end, 1:p.N)';
qdot_rel = y_vec(end, p.N+1:end)';

% Compute ballistic trajectory (pass empty t_span for auto-compute)
traj = ballistic_trajectory(q_rel, qdot_rel, p, []);

%% ── Create figure ─────────────────────────────────────────────────────────
fig = figure('Name', sprintf('Week 2 Optimized Trajectory - %s', upper(config_name)), ...
    'Position', [100, 100, 1200, 800], 'Color', 'w');

%% ── Main animation subplot ────────────────────────────────────────────────
ax_main = subplot(2, 2, [1, 3]);
hold(ax_main, 'on');
axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title(sprintf('Week 2: Single-Shooting Optimized Trajectory (%s)', upper(config_name)));

% Draw environment (obstacles, target, workspace)
draw_environment(ax_main, p);

% Compute landing info
rc = release_condition(q_rel, qdot_rel, p);
fprintf('\n  Optimized Landing: x = %.3f m, miss = %.2f cm\n', ...
    rc.x_land, abs(rc.miss)*100);

%% ── Torque profile subplot ────────────────────────────────────────────────
ax_tau = subplot(2, 2, 2);
hold(ax_tau, 'on');
grid on;
xlabel('Time [s]');
ylabel('Torque [N·m]');
title('Optimized Torque Profiles');

% Plot piecewise-constant torques
M = p.opt.M;
dt_interval = t_release / M;
t_edges = linspace(0, t_release, M+1);

z_mat = reshape(z, [p.N, M]);
colors = lines(p.N);

for i = 1:p.N
    % Plot as stairs (piecewise-constant)
    stairs(ax_tau, t_edges, [z_mat(i,:), z_mat(i,end)], ...
        'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Joint %d', i));
end

% Add torque limits as shaded regions
ylim_vals = [min(p.lim.tau_min)-2, max(p.lim.tau_max)+2];
ylim(ax_tau, ylim_vals);
legend('Location', 'best');

%% ── Joint angles subplot ──────────────────────────────────────────────────
ax_q = subplot(2, 2, 4);
hold(ax_q, 'on');
grid on;
xlabel('Time [s]');
ylabel('Joint Angle [rad]');
title('Joint Angle Trajectories');

for i = 1:p.N
    plot(ax_q, t_vec, y_vec(:, i), 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('q_%d', i));
end

legend('Location', 'best');

%% ── Simple animation loop (non-interactive, just play through) ────────────
n_frames = length(t_vec);
skip = max(1, floor(n_frames / 100));  % Show ~100 frames max

for k = 1:skip:n_frames
    % Update arm visualization in main plot
    q_k = y_vec(k, 1:p.N)';
    
    % Draw arm
    [joint_pos, ~, ~] = forward_kinematics(q_k, p);
    
    % Clear previous arm drawing (if any)
    h_arm = findobj(ax_main, 'Tag', 'arm_links');
    delete(h_arm);
    
    % Draw arm links
    plot(ax_main, joint_pos(1,:), joint_pos(2,:), 'o-', ...
        'Color', p.viz.col.arm_link, 'LineWidth', 3, ...
        'MarkerSize', 8, 'MarkerFaceColor', p.viz.col.arm_joint, ...
        'Tag', 'arm_links');
    
    % Draw object at end-effector
    alpha_N = sum(q_k);
    ee_xy = joint_pos(:, end);
    R_N = [cos(alpha_N), -sin(alpha_N);
           sin(alpha_N),  cos(alpha_N)];
    obj_pos_k = ee_xy + R_N * p.obj.r_gc;
    
    h_obj = findobj(ax_main, 'Tag', 'object');
    delete(h_obj);
    
    if strcmp(p.obj.shape, 'rectangle')
        % Draw rectangle
        hw = p.obj.a;
        hh = p.obj.b;
        corners_local = [hw, hh; -hw, hh; -hw, -hh; hw, -hh; hw, hh]';
        corners_world = R_N * corners_local;
        fill(ax_main, obj_pos_k(1) + corners_world(1,:), ...
            obj_pos_k(2) + corners_world(2,:), ...
            p.viz.col.object, 'EdgeColor', 'k', 'LineWidth', 1.5, 'Tag', 'object');
    else  % circle
        theta_circ = linspace(0, 2*pi, 30);
        xc = obj_pos_k(1) + p.obj.r * cos(theta_circ);
        yc = obj_pos_k(2) + p.obj.r * sin(theta_circ);
        fill(ax_main, xc, yc, p.viz.col.object, 'EdgeColor', 'k', ...
            'LineWidth', 1.5, 'Tag', 'object');
    end
    
    % Update title with current time
    title(ax_main, sprintf('Week 2 Optimized - %s | t = %.2f s', ...
        upper(config_name), t_vec(k)));
    
    drawnow;
    pause(0.02);  % Small pause for animation
end

% Hold final frame briefly
pause(0.5);

fprintf('  Animation complete.\n');

end