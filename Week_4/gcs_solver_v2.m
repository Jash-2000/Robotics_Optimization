function [paths, regions, region_path] = gcs_solver_v2(config_name, p)
% gcs_solver.m  (v2 — reachability-aware)
% =========================================================================
% Graph-based Convex Synthesis (GCS) for workspace path planning.
%
% Solves the pure geometric problem: find collision-free paths through
% obstacle-free convex regions in the REACHABLE workspace.
%
% KEY INSIGHT: The arm is anchored at [0, p.y0] with total reach
%   L_total = sum(p.l). All regions must lie within this reach circle.
%   GCS paths that leave the reachable workspace are useless.
%
% OUTPUTS:
%   paths       — cell array of K candidate workspace paths [T×2] (x,y)
%   regions     — struct array of convex regions
%   region_path — sequence of region indices through the graph
% =========================================================================

fprintf('\n');
fprintf('==================================================================\n');
fprintf('GCS Workspace Path Planner\n');
fprintf('Config: %s\n', config_name);
fprintf('==================================================================\n');

obstacles = load_obstacle_config(config_name, p);

% Arm geometry
base = [0; p.y0];                       % [0, 1.0]
L_total = sum(p.l);                      % 1.13 m
fprintf('Arm base: [%.2f, %.2f], reach: %.2f m\n', base(1), base(2), L_total);

% ── STEP 1: Define obstacle-free convex regions ─────────────────────────
fprintf('\n[1] Defining convex obstacle-free regions...\n');
regions = define_regions(config_name, obstacles, base, L_total);

for i = 1:length(regions)
    fprintf('  R%d (%s): x=[%.2f,%.2f] y=[%.2f,%.2f]\n', ...
        i, regions(i).name, ...
        regions(i).x_min, regions(i).x_max, ...
        regions(i).y_min, regions(i).y_max);
end

% ── STEP 2: Build region adjacency graph ─────────────────────────────────
fprintf('[2] Building adjacency graph...\n');
n = length(regions);
adjacency = zeros(n);
for i = 1:n
    for j = i+1:n
        ci = region_center(regions(i));
        cj = region_center(regions(j));
        d  = norm(cj - ci);
        adjacency(i,j) = d;
        adjacency(j,i) = d;
    end
end

% ── STEP 3: Find start region ────────────────────────────────────────────
fprintf('[3] Finding start region...\n');
[jpos, ~] = forward_kinematics(p.q0, p);
hand = jpos(:, end);
fprintf('  Hand position: [%.3f, %.3f]\n', hand(1), hand(2));

start_idx = [];
for r = 1:n
    if point_in_region(hand, regions(r))
        start_idx = r;
        break;
    end
end
if isempty(start_idx)
    dists = arrayfun(@(r) dist_to_region(hand, r), regions);
    [~, start_idx] = min(dists);
    fprintf('  Hand outside all regions, closest: R%d\n', start_idx);
else
    fprintf('  Hand in R%d\n', start_idx);
end

goal_idx = n;  % last region is pre-release

% ── STEP 4: Dijkstra shortest path ──────────────────────────────────────
fprintf('[4] Dijkstra shortest path...\n');
[region_path, cost] = dijkstra(start_idx, goal_idx, adjacency);

path_str = sprintf('%d', region_path(1));
for r = 2:length(region_path)
    path_str = sprintf('%s -> %d', path_str, region_path(r));
end
fprintf('  Path: %s  (cost %.2f m)\n', path_str, cost);

% ── STEP 5: Generate K candidate workspace paths ────────────────────────
K = 5;
fprintf('[5] Generating %d candidate paths...\n', K);
paths = cell(K, 1);

for k = 1:K
    paths{k} = make_path(region_path, regions, hand, k, base, L_total);
    fprintf('  Path %d: %d waypoints\n', k, size(paths{k}, 1));
end

fprintf('==================================================================\n');
fprintf('GCS complete: %d workspace paths generated\n', K);
fprintf('==================================================================\n');

end

% =========================================================================
% DEFINE REGIONS — all within arm reach
% =========================================================================
function regions = define_regions(config_name, obstacles, base, L)
% Regions must satisfy: every point is within L of base.
% We clip all regions to the reachable annulus.

margin = 0.12;  % 12 cm obstacle clearance

if strcmp(config_name, 'simple')
    obs = obstacles(1);  % cx=1.3, cy=0.9, r=0.20
    
    % NOTE: obs right edge = 1.3+0.20+0.12 = 1.62 > arm reach (1.13m).
    % The arm CANNOT reach to the right of this obstacle.
    % Path: start (left) → above obstacle → pre-release (high).
    
    % R1: Start — left of obstacle
    regions(1) = make_region('start', ...
        -0.5, obs.cx - obs.r - margin, ...
         0.5, base(2) + L);
    
    % R2: Above obstacle
    regions(2) = make_region('above_obs', ...
        obs.cx - obs.r - margin, obs.cx + obs.r + margin, ...
        obs.cy + obs.r + margin, base(2) + L);
    
    % R3: Pre-release — high and slightly right of centre
    regions(3) = make_region('pre_release', ...
        0.3, base(1) + L * 0.9, ...
        base(2) + 0.3, base(2) + L);
    
elseif strcmp(config_name, 'moderate')
    obs1 = obstacles(1);  % cx=1.0, cy=1.1, r=0.25
    obs2 = obstacles(2);  % cx=1.7, cy=1.8, r=0.25
    
    % Strategy: navigate LEFT of obs1, then HIGH above obs1 (left of obs2),
    % then pre-release on the upper-left reachable area.
    % NOTE: the gap between obs1 and obs2 is too small (<0.2m) to route through.
    
    % R1: Start — left of obs1 (full y range left of obs1)
    regions(1) = make_region('start', ...
        -0.5,  obs1.cx - obs1.r - margin, ...
        -0.2,  base(2) + L);
    
    % R2: Below obs1 — under both obstacles
    regions(2) = make_region('below_obs1', ...
        -0.5,  obs1.cx + obs1.r + margin, ...
        -0.2,  obs1.cy - obs1.r - margin);
    
    % R3: Above obs1, left of obs2 — the viable corridor
    %     y_min = obs1 top + margin, y_max = arm reach
    %     x_max = left edge of obs2 - margin (must clear obs2 horizontally)
    y3_min = obs1.cy + obs1.r + margin;          % 1.47
    y3_max = base(2) + L - 0.05;                 % 2.08 (leave 5cm margin from reach limit)
    x3_max = obs2.cx - obs2.r - margin;          % 1.33
    if y3_min < y3_max && x3_max > -0.2
        regions(3) = make_region('above_obs1_left_obs2', ...
            -0.2, x3_max, y3_min, y3_max);
    else
        % Fallback: just use left corridor higher up
        regions(3) = make_region('left_high', ...
            -0.5, obs1.cx - obs1.r - margin, ...
            obs1.cy, base(2) + L - 0.05);
    end
    
    % R4: Pre-release — left and high, within arm reach
    %     Avoid being too close to obs2 (x < obs2.cx - obs2.r - margin)
    y4_min = obs1.cy + obs1.r;                   % 1.35 — above obs1 top
    y4_max = base(2) + L - 0.05;                 % 2.08
    regions(4) = make_region('pre_release', ...
        -0.3, min(x3_max, base(1) + L * 0.7), ...
        y4_min, y4_max);
    
else  % hard
    obs1 = obstacles(1);  % cx=0.8, cy=0.8, r=0.30
    obs2 = obstacles(2);  % cx=1.4, cy=1.5, r=0.20
    obs3 = obstacles(3);  % cx=1.9, cy=2.2, r=0.25 — BARELY reachable
    
    % R1: Start — left of obs1
    regions(1) = make_region('start', ...
        -0.5, obs1.cx - obs1.r - margin, ...
        0.0, base(2) + L);
    
    % R2: Above obs1, left of obs2
    regions(2) = make_region('above_obs1', ...
        -0.3, obs2.cx - obs2.r - margin, ...
        obs1.cy + obs1.r + margin, base(2) + L);
    
    % R3: Below obs1 (low corridor)
    regions(3) = make_region('below_obs1', ...
        -0.3, obs1.cx + obs1.r + margin, ...
        0.0, obs1.cy - obs1.r - margin);
    
    % R4: Pre-release — high, near base (above obs2)
    regions(4) = make_region('pre_release', ...
        -0.2, obs2.cx - obs2.r - margin, ...
        obs2.cy + obs2.r + margin, base(2) + L);
end

% Clip all regions to reachable workspace
for i = 1:length(regions)
    regions(i).x_max = min(regions(i).x_max, base(1) + L);
    regions(i).x_min = max(regions(i).x_min, base(1) - L);
    regions(i).y_max = min(regions(i).y_max, base(2) + L);
    regions(i).y_min = max(regions(i).y_min, base(2) - L);
end

% Validate all regions — catch degenerate definitions early
for i = 1:length(regions)
    if regions(i).x_min >= regions(i).x_max || regions(i).y_min >= regions(i).y_max
        error('Region %d (%s) is degenerate: x=[%.3f,%.3f] y=[%.3f,%.3f]', ...
            i, regions(i).name, ...
            regions(i).x_min, regions(i).x_max, ...
            regions(i).y_min, regions(i).y_max);
    end
end

end

function r = make_region(name, xmin, xmax, ymin, ymax)
r.name  = name;
r.x_min = xmin;  r.x_max = xmax;
r.y_min = ymin;  r.y_max = ymax;
end

function c = region_center(r)
c = [mean([r.x_min, r.x_max]); mean([r.y_min, r.y_max])];
end

function b = point_in_region(pt, r)
b = pt(1) >= r.x_min && pt(1) <= r.x_max && ...
    pt(2) >= r.y_min && pt(2) <= r.y_max;
end

function d = dist_to_region(pt, r)
dx = max([r.x_min - pt(1), 0, pt(1) - r.x_max]);
dy = max([r.y_min - pt(2), 0, pt(2) - r.y_max]);
d = sqrt(dx^2 + dy^2);
end

% =========================================================================
% DIJKSTRA
% =========================================================================
function [path, cost] = dijkstra(s, g, adj)
n = size(adj,1);
dist = inf(n,1);  dist(s) = 0;
parent = zeros(n,1);
visited = false(n,1);

for iter = 1:n
    candidates = dist;
    candidates(visited) = inf;
    [~, u] = min(candidates);
    if isinf(dist(u)), break; end
    visited(u) = true;
    for v = 1:n
        if adj(u,v) > 0 && ~visited(v)
            alt = dist(u) + adj(u,v);
            if alt < dist(v)
                dist(v) = alt;
                parent(v) = u;
            end
        end
    end
end

path = [];
cur = g;
while cur ~= 0
    path = [cur; path];
    cur = parent(cur);
end
cost = dist(g);
end

% =========================================================================
% GENERATE WORKSPACE PATH
% =========================================================================
function path = make_path(region_path, regions, start_pos, variant, base, L)
% Build waypoints through region centers, clipped to reach circle

waypoints = [start_pos'];

for idx = 1:length(region_path)
    r = regions(region_path(idx));
    cx = (r.x_min + r.x_max) / 2;
    cy = (r.y_min + r.y_max) / 2;
    
    % Small perturbation for path diversity
    wx = (r.x_max - r.x_min) / 4;
    wy = (r.y_max - r.y_min) / 4;
    px = wx * 0.25 * sin(variant * 1.3 + idx * 0.7);
    py = wy * 0.25 * cos(variant * 0.9 + idx * 1.1);
    
    wp = [cx + px, cy + py];
    
    % Clip to reachable workspace (distance from base < 0.95*L for safety)
    d = norm(wp' - base);
    if d > 0.95 * L
        wp = base' + (wp - base') * (0.95 * L / d);
    end
    
    waypoints = [waypoints; wp];
end

% Smooth with cubic spline
t_wp   = linspace(0, 1, size(waypoints, 1));
t_fine = linspace(0, 1, 30);
path   = interp1(t_wp, waypoints, t_fine, 'pchip');

end