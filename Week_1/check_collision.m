function [collides, details] = check_collision(query, obstacles, p, margin)
% check_collision.m
% =========================================================================
% Checks whether a query shape collides with any obstacle in the scene.
%
% The function supports two query types:
%
%   1. ARM LINK (capsule / line segment):
%      Checks whether the swept line segment of a robot link (from its
%      proximal joint to distal joint) collides with any obstacle.
%      A link is modelled as a ZERO-RADIUS line segment for the base check;
%      no explicit link thickness is assumed (conservative = thin).
%      Algorithm: point-to-segment distance (circle), AABB intersection (rect)
%
%   2. OBJECT (oriented bounding shape rotating in flight):
%      The object bounding shape (circle or rectangle) at a given world
%      position and orientation angle is checked against every obstacle.
%      Algorithm: circle-circle distance, rect-circle closest point,
%                 SAT (Separating Axis Theorem) for rect-rect
%
% INPUTS:
%   query     – struct describing what to check:
%     For arm link:
%       .type   = 'link'
%       .p1     = [2×1] proximal joint world position [m]
%       .p2     = [2×1] distal  joint world position  [m]
%     For object:
%       .type   = 'object'
%       .pos    = [2×1] object CoM world position [m]
%       .theta  = scalar  object orientation [rad]
%
%   obstacles – obstacle struct array from load_obstacle_config.m
%               (empty struct array = no obstacles)
%   p         – parameter struct from params.m
%   margin    – (optional) safety margin [m] (default: 0)
%               Collision if dist < -margin (penetration beyond margin)
%
% OUTPUTS:
%   collides  – logical scalar: true if ANY collision detected
%   details   – struct array (one entry per obstacle tested):
%       .obs_idx  – obstacle index
%       .collides – logical
%       .dist     – minimum separation distance [m] (negative = penetration)
%
% =========================================================================

% Handle optional margin parameter
if nargin < 4
    margin = 0;
end

collides = false;
details  = struct('obs_idx',{},'collides',{},'dist',{});

if isempty(obstacles)
    return;
end

n_obs = numel(obstacles);

for k = 1:n_obs
    obs = obstacles(k);
    hit  = false;
    dist = Inf;

    switch query.type

        %% ── ARM LINK (segment vs obstacle) ──────────────────────────────
        case 'link'
            p1 = query.p1;
            p2 = query.p2;

            switch obs.type
                case 'circle'
                    % Minimum distance from segment p1-p2 to circle centre
                    d_seg = point_to_segment_dist([obs.cx; obs.cy], p1, p2);
                    dist  = d_seg - obs.r;
                    hit   = dist < -margin;

                case 'rectangle'
                    % Rotate segment into rectangle's local frame, then do
                    % AABB (axis-aligned bounding box) segment test
                    c   = [obs.cx; obs.cy];
                    phi = obs.phi;
                    R   = [cos(phi), sin(phi); -sin(phi), cos(phi)];  % world→local
                    p1l = R*(p1 - c);
                    p2l = R*(p2 - c);
                    % Check segment vs axis-aligned rect [-hw,hw]×[-hh,hh]
                    [hit_r, dist_r] = segment_vs_aabb(p1l, p2l, obs.hw, obs.hh);
                    hit  = hit_r;
                    dist = dist_r;
            end

        %% ── OBJECT (oriented bounding shape vs obstacle) ─────────────────
        case 'object'
            obj_pos   = query.pos;    % [2×1] CoM in world
            obj_theta = query.theta;  % orientation [rad]

            switch obs.type
                case 'circle'
                    % Distance between object and obstacle shapes
                    switch p.obj.shape
                        case 'circle'
                            % Circle vs circle
                            d_centres = norm(obj_pos - [obs.cx; obs.cy]);
                            dist = d_centres - p.obj.r - obs.r;
                            hit  = dist < -margin;

                        case 'rectangle'
                            % Oriented rectangle vs circle:
                            % Transform circle centre into rectangle's frame
                            c_obs   = [obs.cx; obs.cy];
                            R_obj   = [cos(obj_theta), sin(obj_theta);
                                      -sin(obj_theta), cos(obj_theta)];
                            c_local = R_obj * (c_obs - obj_pos);
                            % Clamp to rectangle extents to find closest point
                            clamped = [clamp(c_local(1), -p.obj.a, p.obj.a);
                                       clamp(c_local(2), -p.obj.b, p.obj.b)];
                            dist = norm(c_local - clamped) - obs.r;
                            hit  = dist < -margin;
                    end

                case 'rectangle'
                    switch p.obj.shape
                        case 'circle'
                            % Circle vs oriented rectangle
                            c_obj   = obj_pos;
                            c_obs   = [obs.cx; obs.cy];
                            phi     = obs.phi;
                            R_obs   = [cos(phi), sin(phi);
                                      -sin(phi), cos(phi)];
                            p_local = R_obs * (c_obj - c_obs);
                            clamped = [clamp(p_local(1), -obs.hw, obs.hw);
                                       clamp(p_local(2), -obs.hh, obs.hh)];
                            dist = norm(p_local - clamped) - p.obj.r;
                            hit  = dist < -margin;

                        case 'rectangle'
                            % Oriented rectangle vs oriented rectangle
                            % Use Separating Axis Theorem (SAT) in 2D
                            [hit_sat, dist_sat] = sat_rect_rect( ...
                                obj_pos, obj_theta, p.obj.a, p.obj.b, ...
                                [obs.cx; obs.cy], obs.phi, obs.hw, obs.hh);
                            hit  = hit_sat || (dist_sat < -margin);
                            dist = dist_sat;
                    end
            end

        otherwise
            error('check_collision: unknown query type "%s"', query.type);
    end

    details(k).obs_idx  = k;
    details(k).collides = hit;
    details(k).dist     = dist;

    if hit
        collides = true;
    end
end

end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: point-to-segment minimum distance
%% ═══════════════════════════════════════════════════════════════════════════
function d = point_to_segment_dist(pt, p1, p2)
% Minimum Euclidean distance from point pt to line segment p1-p2.
v  = p2 - p1;
w  = pt - p1;
t  = dot(w, v) / max(dot(v, v), 1e-12);
t  = max(0, min(1, t));
closest = p1 + t * v;
d = norm(pt - closest);
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: segment vs axis-aligned bounding box
%% ═══════════════════════════════════════════════════════════════════════════
function [hit, dist] = segment_vs_aabb(p1, p2, hw, hh)
% Checks if segment p1-p2 (in rectangle's local frame) intersects
% the AABB [-hw,hw] x [-hh,hh].
% Returns hit = true and approximate dist (negative on penetration).

% Quick bounding box rejection
if min(p1(1),p2(1)) > hw || max(p1(1),p2(1)) < -hw || ...
   min(p1(2),p2(2)) > hh || max(p1(2),p2(2)) < -hh

    % Both endpoints outside - compute min dist to box
    d1 = point_to_aabb_dist(p1, hw, hh);
    d2 = point_to_aabb_dist(p2, hw, hh);
    dist = min(d1, d2);
    hit  = false;
    return;
end

% Check if either endpoint is inside
if point_in_aabb(p1, hw, hh) || point_in_aabb(p2, hw, hh)
    hit  = true;
    dist = -min(point_to_aabb_dist(p1,hw,hh), point_to_aabb_dist(p2,hw,hh));
    return;
end

% Parametric segment-edge intersection (4 box edges)
hit  = false;
dist = Inf;
edges = [ hw, -hh,  hw,  hh;   % right edge
         -hw, -hh, -hw,  hh;   % left edge
         -hw,  hh,  hw,  hh;   % top edge
         -hw, -hh,  hw, -hh];  % bottom edge

for e = 1:4
    ep1 = edges(e,1:2)';
    ep2 = edges(e,3:4)';
    [inter, ~] = segment_segment_intersect(p1, p2, ep1, ep2);
    if inter
        hit  = true;
        dist = 0;
        return;
    end
    % Compute distance from segment to this edge (for accurate dist when no intersection)
    d_seg_to_edge = min([point_to_segment_dist(ep1, p1, p2), ...
                         point_to_segment_dist(ep2, p1, p2), ...
                         point_to_segment_dist(p1, ep1, ep2), ...
                         point_to_segment_dist(p2, ep1, ep2)]);
    dist = min(dist, d_seg_to_edge);
end
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: segment-segment intersection test
%% ═══════════════════════════════════════════════════════════════════════════
function [inter, t] = segment_segment_intersect(p1, p2, p3, p4)
d1 = p2 - p1;
d2 = p4 - p3;
cross_d = d1(1)*d2(2) - d1(2)*d2(1);
t  = NaN;
inter = false;
if abs(cross_d) < 1e-12, return; end
t  = ((p3(1)-p1(1))*d2(2) - (p3(2)-p1(2))*d2(1)) / cross_d;
u  = ((p3(1)-p1(1))*d1(2) - (p3(2)-p1(2))*d1(1)) / cross_d;
inter = (t >= 0 && t <= 1 && u >= 0 && u <= 1);
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: point in AABB
%% ═══════════════════════════════════════════════════════════════════════════
function inside = point_in_aabb(pt, hw, hh)
inside = abs(pt(1)) <= hw && abs(pt(2)) <= hh;
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: point to AABB distance
%% ═══════════════════════════════════════════════════════════════════════════
function d = point_to_aabb_dist(pt, hw, hh)
dx = max(0, abs(pt(1)) - hw);
dy = max(0, abs(pt(2)) - hh);
d  = sqrt(dx^2 + dy^2);
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: SAT for oriented rectangle vs oriented rectangle
%% ═══════════════════════════════════════════════════════════════════════════
function [hit, penetration_depth] = sat_rect_rect(c1, phi1, hw1, hh1, ...
                                                   c2, phi2, hw2, hh2)
% Separating Axis Theorem for two oriented rectangles in 2D.
% Returns hit=true if they overlap, plus signed penetration depth.

axes_1 = [cos(phi1), sin(phi1); -sin(phi1), cos(phi1)]';  % 2 cols = 2 axes
axes_2 = [cos(phi2), sin(phi2); -sin(phi2), cos(phi2)]';

test_axes = [axes_1, axes_2];   % 2×4 matrix, each col is a separating axis candidate

half_extents_1 = [hw1; hh1];
half_extents_2 = [hw2; hh2];

d = c2 - c1;   % vector between centres

min_overlap = Inf;
hit = true;

for i = 1:4
    ax  = test_axes(:,i);   % unit separating axis candidate
    ax  = ax / max(norm(ax), 1e-12);

    % Project each rectangle onto ax (support function)
    proj_d  = abs(dot(d, ax));
    proj_1  = abs(dot(axes_1(:,1)*hw1, ax)) + abs(dot(axes_1(:,2)*hh1, ax));
    proj_2  = abs(dot(axes_2(:,1)*hw2, ax)) + abs(dot(axes_2(:,2)*hh2, ax));

    overlap = proj_1 + proj_2 - proj_d;
    if overlap < 0
        hit = false;
        penetration_depth = overlap;   % negative = separation
        return;
    end
    min_overlap = min(min_overlap, overlap);
end

penetration_depth = -min_overlap;   % negative = penetrating
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: scalar clamp
%% ═══════════════════════════════════════════════════════════════════════════
function y = clamp(x, lo, hi)
y = max(lo, min(hi, x));
end
