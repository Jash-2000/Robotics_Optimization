function rc = release_condition(q_rel, qdot_rel, p)
% release_condition.m
% =========================================================================
% Computes the release state and predicted landing location for the thrown
% object. This is a STANDALONE function called by:
%   - Week 1: ballistic_trajectory.m (for trajectory computation)
%   - Week 2+: shooting_objective.m (for optimization objective evaluation)
%
% INPUTS:
%   q_rel    – [N×1] joint angles at the moment of release [rad]
%   qdot_rel – [N×1] joint velocities at release [rad/s]
%   p        – parameter struct from params.m
%
% OUTPUTS:  rc  – struct with fields:
%   .obj_pos    [2×1]  object CoM position at release [m]
%   .obj_vel    [2×1]  object CoM velocity at release [m/s]
%   .obj_theta  scalar object orientation at release [rad]
%   .obj_omega  scalar object angular velocity at release [rad/s]
%   .speed      scalar release speed |v_obj| [m/s]
%   .angle_deg  scalar release angle from horizontal [deg]
%   .t_land     scalar time to ground impact [s]
%   .x_land     scalar landing x-coordinate [m]
%   .miss       scalar signed miss distance: x_land - d [m]
%   .miss_abs   scalar absolute miss distance [m]
%
% PHYSICS:
%   Object CoM position:  p_obj = p_ee + R(alpha_N) * r_gc
%   Object CoM velocity:  v_obj = J_obj_lin * qdot_rel
%   Object ang. velocity: omega = J_obj_ang * qdot_rel = sum(qdot_rel)
%   Ballistic landing:    analytic projectile motion with gravity
% =========================================================================

g = p.g;

%% ── Release state (forward kinematics + Jacobians) ───────────────────────
[~, ~, ee_xy] = forward_kinematics(q_rel, p);

% Object CoM position at release (in world frame)
alpha_N = sum(q_rel);   % absolute angle of last link
R_N = [cos(alpha_N), -sin(alpha_N);
       sin(alpha_N),  cos(alpha_N)];
obj_pos = ee_xy + R_N * p.obj.r_gc;        % [2×1] world frame

% Object CoM velocity at release via Jacobian
[~, ~, J_obj_lin, J_obj_ang] = jacobians(q_rel, p);
obj_vel = J_obj_lin * qdot_rel;            % [2×1] m/s
obj_omega = J_obj_ang * qdot_rel;          % scalar rad/s (constant in flight)

% Object orientation at release = absolute angle of last link
obj_theta = alpha_N;

% Release speed and angle
speed = norm(obj_vel);
angle_deg = atan2d(obj_vel(2), obj_vel(1));

%% ── Ballistic landing prediction ─────────────────────────────────────────
% Solve for time when object hits ground (y_com = 0):
%   y(t) = obj_pos(2) + obj_vel(2)*t - 0.5*g*t^2 = 0
%   This is a quadratic: a*t^2 + b*t + c = 0
a_coef = -0.5 * g;
b_coef =  obj_vel(2);
c_coef =  obj_pos(2);

discriminant = b_coef^2 - 4*a_coef*c_coef;

if discriminant < 0
    % Object never reaches ground (thrown downward too slowly or upward)
    % This shouldn't happen in typical throwing, but handle gracefully
    t_land = NaN;
    x_land = NaN;
    miss   = NaN;
    miss_abs = NaN;
else
    % Two roots; take the smallest positive one (first ground impact)
    t_roots = [(-b_coef + sqrt(discriminant)); ...
               (-b_coef - sqrt(discriminant))] / (2*a_coef);
    t_land_candidates = t_roots(t_roots > 1e-6);   % positive times only
    
    if isempty(t_land_candidates)
        % No positive root (object moving away from ground)
        t_land = NaN;
        x_land = NaN;
        miss   = NaN;
        miss_abs = NaN;
    else
        t_land = min(t_land_candidates);
        x_land = obj_pos(1) + obj_vel(1) * t_land;
        miss   = x_land - p.task.d;      % signed: positive = overshoot
        miss_abs = abs(miss);
    end
end

%% ── Pack output ──────────────────────────────────────────────────────────
rc.obj_pos    = obj_pos;
rc.obj_vel    = obj_vel;
rc.obj_theta  = obj_theta;
rc.obj_omega  = obj_omega;
rc.speed      = speed;
rc.angle_deg  = angle_deg;
rc.t_land     = t_land;
rc.x_land     = x_land;
rc.miss       = miss;
rc.miss_abs   = miss_abs;

end
