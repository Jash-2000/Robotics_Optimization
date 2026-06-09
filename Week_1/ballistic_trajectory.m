function traj = ballistic_trajectory(q_rel, qdot_rel, p, t_span)
% ballistic_trajectory.m
% =========================================================================
% Computes the analytic ballistic trajectory of the object after release
% from the gripper. No air drag. Gravity only.
%
% The object's CoM follows projectile motion. The object ROTATES at constant
% angular velocity (no external torques in flight). Its oriented bounding
% shape (circle or rectangle) is tracked at each time step for collision
% checking against flight-path obstacles.
%
% INPUTS:
%   q_rel    – [N×1] joint angles at the moment of release [rad]
%   qdot_rel – [N×1] joint velocities at release [rad/s]
%   p        – parameter struct from params.m
%   t_span   – [1×2] time span for ballistic phase [t_release, t_end]
%              (or empty → auto-computed until ground impact)
%
% OUTPUTS:  traj  – struct with fields:
%   .t          [K×1]  time samples [s]
%   .pos        [2×K]  object CoM world position [m]
%   .vel        [2×K]  object CoM velocity [m/s]
%   .theta      [1×K]  object orientation [rad]  (0 at release)
%   .omega      scalar  object angular velocity [rad/s]  (constant)
%   .landed     logical – true if object hits ground (y_com <= 0) in span
%   .t_land     scalar  – time of ground impact [s]  (NaN if not landed)
%   .x_land     scalar  – x position of ground impact [m] (NaN if not landed)
%
% PHYSICS:
%   CoM position:    r(t) = r0 + v0*t + 0.5*[0;-g]*t^2
%   CoM velocity:    v(t) = v0 + [0;-g]*t
%   Orientation:     theta(t) = theta0 + omega * t
%   Angular vel:     omega = constant (no air torque)
% =========================================================================

N  = p.N;
g  = p.g;

%% ── Release state via release_condition.m ────────────────────────────────
rc = release_condition(q_rel, qdot_rel, p);

r0     = rc.obj_pos;     % [2×1] initial position
v0     = rc.obj_vel;     % [2×1] initial velocity
omega  = rc.obj_omega;   % scalar angular velocity
theta0 = rc.obj_theta;   % initial orientation

%% ── Time vector ──────────────────────────────────────────────────────────
if isempty(t_span)
    % Use analytic landing time from release_condition
    if isnan(rc.t_land)
        error('ballistic_trajectory: object never reaches ground.');
    end
    t_land_val = rc.t_land;
    t_vec = linspace(0, t_land_val * 1.05, 300)';
else
    t_vec = (t_span(1) : 0.005 : t_span(2))' - t_span(1);
    t_land_val = rc.t_land;  % may be NaN if not landing in span
end

%% ── Analytic trajectory ──────────────────────────────────────────────────
K   = length(t_vec);
pos = zeros(2, K);
vel = zeros(2, K);
theta_vec = zeros(1, K);

for k = 1:K
    t = t_vec(k);
    pos(:,k)    = r0 + v0*t + 0.5*[0; -g]*t^2;
    vel(:,k)    = v0 + [0; -g]*t;
    theta_vec(k) = theta0 + omega * t;
end

%% ── Ground impact detection ──────────────────────────────────────────────
below_ground = pos(2,:) <= 0;
if any(below_ground)
    traj.landed = true;
    k_land = find(below_ground, 1);
    % Refine landing time via linear interpolation
    if k_land > 1
        y1 = pos(2, k_land-1);  t1 = t_vec(k_land-1);
        y2 = pos(2, k_land  );  t2 = t_vec(k_land  );
        t_land_val = t1 + (0 - y1)*(t2-t1)/(y2-y1);
    else
        t_land_val = t_vec(k_land);
    end
    traj.t_land = t_land_val + (isempty(t_span)*0);
    
    % Use release_condition's x_land if available (more accurate)
    if ~isnan(rc.x_land)
        traj.x_land = rc.x_land;
    else
        traj.x_land = r0(1) + v0(1)*t_land_val;
    end
    
    % Trim trajectory at ground level
    t_vec     = t_vec(1:k_land);
    pos       = pos(:,  1:k_land);
    vel       = vel(:,  1:k_land);
    theta_vec = theta_vec(1:k_land);
else
    traj.landed = false;
    traj.t_land = NaN;
    traj.x_land = NaN;
end

%% ── Pack output ──────────────────────────────────────────────────────────
traj.t     = t_vec;
traj.pos   = pos;
traj.vel   = vel;
traj.theta = theta_vec;
traj.omega = omega;
traj.r0    = r0;
traj.v0    = v0;

%% ── Print summary ────────────────────────────────────────────────────────
fprintf('\n--- Ballistic Trajectory Summary ---\n');
fprintf('  Release position  : (%.3f, %.3f) m\n', r0(1), r0(2));
fprintf('  Release velocity  : (%.3f, %.3f) m/s\n', v0(1), v0(2));
fprintf('  Release speed     : %.3f m/s\n', norm(v0));
fprintf('  Release angle     : %.2f deg from horizontal\n', ...
    atan2d(v0(2), v0(1)));
fprintf('  Angular velocity  : %.3f rad/s\n', omega);
if traj.landed
    fprintf('  Landing time      : %.3f s after release\n', traj.t_land);
    fprintf('  Landing x         : %.3f m from base\n', traj.x_land);
    fprintf('  Target x          : %.3f m from base\n', p.task.d);
    fprintf('  Miss distance     : %.4f m\n', abs(traj.x_land - p.task.d));
end
fprintf('------------------------------------\n\n');

end
