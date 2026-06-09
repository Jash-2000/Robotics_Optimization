function ax = draw_environment(p, obstacles)
% draw_environment.m
% =========================================================================
% Draws the static simulation environment: ground, robot base marker,
% target, workspace extent, and axis labels. Optionally draws obstacles.
%
% INPUTS:
%   p         – parameter struct from params.m
%   obstacles – (optional) obstacle struct array to draw on static figure
%               (empty or omitted = no obstacles drawn)
%
% OUTPUT:
%   ax – handle to the main simulation axes
% =========================================================================

% Handle optional obstacles argument
if nargin < 2
    obstacles = [];
end

%% ── Figure setup ─────────────────────────────────────────────────────────
fig = figure('Name','Robotic Arm Throwing Simulation', ...
    'Position',[100, 80, p.viz.fig_width, p.viz.fig_height], ...
    'Color','white');

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
ax.GridAlpha = 0.15;
ax.Box = 'on';
ax.FontSize = 11;

%% ── Axis limits ──────────────────────────────────────────────────────────
arm_reach   = sum(p.l);                   % maximum arm reach from base
x_min       = -arm_reach - p.viz.axis_margin;
x_max       =  p.task.d  + p.viz.axis_margin;
y_min       = -0.15;
y_max       =  p.y0 + arm_reach + p.viz.axis_margin;

axis(ax, [x_min, x_max, y_min, y_max]);
daspect(ax, [1 1 1]);
xlabel(ax, 'x  [m]', 'FontSize', 12);
ylabel(ax, 'y  [m]', 'FontSize', 12);
title(ax,  '4-Link Planar Robotic Arm — Throwing Simulation', 'FontSize', 13);

%% ── Ground line ──────────────────────────────────────────────────────────
plot(ax, [x_min, x_max], [0, 0], '-', ...
    'Color', p.viz.col.ground, 'LineWidth', 2.5);

% Ground hatch fill (visual depth)
hatch_x = x_min : 0.3 : x_max;
for hx = hatch_x
    plot(ax, [hx, hx+0.15], [0, -0.12], '-', ...
        'Color', p.viz.col.ground*0.7, 'LineWidth', 0.8);
end

%% ── Robot base ───────────────────────────────────────────────────────────
% Pedestal from ground to base height
plot(ax, [0, 0], [0, p.y0], '-', ...
    'Color', [0.3 0.3 0.3], 'LineWidth', 4);
% Base marker
plot(ax, 0, p.y0, 's', ...
    'MarkerSize', 14, 'MarkerFaceColor', [0.3 0.3 0.3], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(ax, 0.06, p.y0, sprintf('Base (y_0 = %.2f m)', p.y0), ...
    'FontSize', 9, 'Color', [0.2 0.2 0.2]);

%% ── Target marker ────────────────────────────────────────────────────────
% Star at (d, 0)
plot(ax, p.task.d, 0, 'p', ...
    'MarkerSize', 20, 'MarkerFaceColor', p.viz.col.target, ...
    'MarkerEdgeColor', p.viz.col.target * 0.6, 'LineWidth', 1.5);
text(ax, p.task.d, 0.12, sprintf('Target\n(%.2f m)', p.task.d), ...
    'HorizontalAlignment','center','FontSize', 9, ...
    'Color', p.viz.col.target, 'FontWeight', 'bold');

%% ── Workspace circle (informational) ────────────────────────────────────
theta_ws = linspace(0, 2*pi, 200);
r_ws     = arm_reach;
plot(ax, r_ws*cos(theta_ws), p.y0 + r_ws*sin(theta_ws), '--', ...
    'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
text(ax, -r_ws*0.7, p.y0 + r_ws*1.02, 'Max. reach', ...
    'FontSize', 8, 'Color', [0.6 0.6 0.6]);

%% ── Release time annotation placeholder ─────────────────────────────────
annotation(fig, 'textbox', [0.01, 0.01, 0.30, 0.07], ...
    'String', sprintf('Release time: %.2f s\nTarget dist: %.2f m', ...
                      p.sim.t_release, p.task.d), ...
    'FitBoxToText','on','BackgroundColor','white', ...
    'EdgeColor',[0.7 0.7 0.7],'FontSize',9);

%% ── Draw obstacles if provided ───────────────────────────────────────────
if ~isempty(obstacles)
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
end

drawnow;

end
