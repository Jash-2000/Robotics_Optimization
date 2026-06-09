function plot_obstacle_configs(p)
% plot_obstacle_configs.m
% =========================================================================
% Creates a side-by-side visualization of the three fixed obstacle
% configurations used for reproducible testing across all methods.
%
% INPUTS:
%   p  – parameter struct from params.m (contains p.configs.*)
%
% OUTPUT:
%   Figure with 1×3 subplots showing Simple, Moderate, and Hard configs
% =========================================================================

config_names = {'simple', 'moderate', 'hard'};
titles_full  = {'Config 1: Simple (Baseline)', ...
                'Config 2: Moderate (Corridor)', ...
                'Config 3: Hard (Slalom)'};

figure('Name', 'Fixed Obstacle Configurations', ...
    'Position', [100, 300, 1400, 400], 'Color', 'white');

for c = 1:3
    subplot(1, 3, c);
    ax = gca;
    hold(ax, 'on');
    grid(ax, 'on');
    ax.GridAlpha = 0.15;
    ax.Box = 'on';
    ax.FontSize = 10;
    
    config_name = config_names{c};
    obstacles = p.configs.(config_name);
    
    % Axis limits
    arm_reach = sum(p.l);
    x_min = -0.5;
    x_max = p.task.d + 0.5;
    y_min = -0.15;
    y_max = p.y0 + arm_reach + 0.3;
    
    axis(ax, [x_min, x_max, y_min, y_max]);
    daspect(ax, [1 1 1]);
    xlabel(ax, 'x [m]', 'FontSize', 10);
    ylabel(ax, 'y [m]', 'FontSize', 10);
    title(ax, titles_full{c}, 'FontSize', 11, 'FontWeight', 'bold');
    
    % Ground
    plot(ax, [x_min, x_max], [0, 0], '-', 'Color', p.viz.col.ground, 'LineWidth', 2);
    
    % Base pedestal
    plot(ax, [0, 0], [0, p.y0], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 3);
    plot(ax, 0, p.y0, 's', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.3 0.3], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1);
    text(ax, 0.05, p.y0 - 0.08, 'Base', 'FontSize', 8, 'Color', [0.2 0.2 0.2]);
    
    % Target
    plot(ax, p.task.d, 0, 'p', 'MarkerSize', 16, 'MarkerFaceColor', p.viz.col.target, ...
        'MarkerEdgeColor', p.viz.col.target * 0.6, 'LineWidth', 1.2);
    text(ax, p.task.d, 0.15, sprintf('Target\n(%.1f m)', p.task.d), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', p.viz.col.target, ...
        'FontWeight', 'bold');
    
    % Workspace circle
    theta_ws = linspace(0, 2*pi, 200);
    plot(ax, arm_reach*cos(theta_ws), p.y0 + arm_reach*sin(theta_ws), '--', ...
        'Color', [0.7 0.7 0.7], 'LineWidth', 0.6);
    
    % Obstacles
    n_obs = numel(obstacles);
    for k = 1:n_obs
        obs = obstacles(k);
        
        switch obs.type
            case 'circle'
                theta_circ = linspace(0, 2*pi, 100);
                xc = obs.cx + obs.r*cos(theta_circ);
                yc = obs.cy + obs.r*sin(theta_circ);
                fill(ax, xc, yc, p.viz.col.obstacle_c, 'FaceAlpha', 0.5, ...
                    'EdgeColor', p.viz.col.obstacle_c*0.7, 'LineWidth', 1.5);
                text(ax, obs.cx, obs.cy, sprintf('O%d', k), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, ...
                    'FontWeight', 'bold', 'Color', 'white');
                
            case 'rectangle'
                hw = obs.hw;
                hh = obs.hh;
                phi = obs.phi;
                corners_local = [ hw,  hh;
                                 -hw,  hh;
                                 -hw, -hh;
                                  hw, -hh;
                                  hw,  hh ]';
                R_phi = [cos(phi), -sin(phi); sin(phi), cos(phi)];
                corners_world = R_phi * corners_local;
                fill(ax, obs.cx + corners_world(1,:), obs.cy + corners_world(2,:), ...
                    p.viz.col.obstacle_r, 'FaceAlpha', 0.5, ...
                    'EdgeColor', p.viz.col.obstacle_r*0.7, 'LineWidth', 1.5);
                text(ax, obs.cx, obs.cy, sprintf('O%d', k), ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, ...
                    'FontWeight', 'bold', 'Color', 'white');
        end
    end
    
    % Annotation with obstacle count
    text(ax, x_min + 0.1, y_max - 0.2, sprintf('%d obstacle(s)', n_obs), ...
        'FontSize', 9, 'BackgroundColor', 'white', 'EdgeColor', [0.7 0.7 0.7], ...
        'Margin', 3);
end

sgtitle('Fixed Obstacle Configurations for Reproducible Testing', ...
    'FontSize', 13, 'FontWeight', 'bold');

drawnow;

end
