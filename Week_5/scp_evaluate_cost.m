function [J_total, cost_info] = scp_evaluate_cost(x_traj, u_traj, t_f, p, obstacles, scp)
% scp_evaluate_cost.m
% =========================================================================
% Evaluate the TRUE (nonlinear) cost of a trajectory.
%
% J = w_pos * (x_land - d)^2
%   + w_energy * sum_k ||u_k||^2 * h
%   + w_collision * collision_penalty
%
% This is called after the QP solve to compute the actual cost reduction
% for the trust-region ratio.
%
% INPUTS:
%   x_traj    — [8×M] state trajectory
%   u_traj    — [4×M] control trajectory
%   t_f       — scalar release time
%   p         — parameter struct
%   obstacles — obstacle struct array
%   scp       — SCP parameter struct
%
% OUTPUTS:
%   J_total   — scalar total cost
%   cost_info — struct with component costs
% =========================================================================

M  = scp.M;
N  = p.N;
h  = t_f / M;

% ── 1. Position cost ─────────────────────────────────────────────────
q_M    = x_traj(1:N, M);
qdot_M = x_traj(N+1:end, M);
rc = release_condition(q_M, qdot_M, p);
x_land = rc.x_land;

if isnan(x_land)
    J_pos = 1e6;         % heavy penalty if landing prediction fails
    x_land = 0;
else
    J_pos = scp.w_position * (x_land - p.task.d)^2;
end

% ── 2. Energy cost (torque squared) ──────────────────────────────────
J_energy = 0;
for k = 1:M
    J_energy = J_energy + scp.w_energy * (u_traj(:,k)' * u_traj(:,k)) * h;
end

% ── 3. Collision penalty (quartic) ───────────────────────────────────
J_coll = 0;
n_collisions = 0;
n_obs = numel(obstacles);

for k = 1:M
    q_k = x_traj(1:N, k);
    [jpos_k, ~, ee_k] = forward_kinematics(q_k, p);

    % Object position
    alpha_N = sum(q_k);
    R_N = [cos(alpha_N), -sin(alpha_N); sin(alpha_N), cos(alpha_N)];
    obj_k = ee_k + R_N * p.obj.r_gc;

    % Check link midpoints + object
    pts = zeros(2, N+1);
    for i = 1:N
        pts(:, i) = 0.5*(jpos_k(:,i) + jpos_k(:,i+1));
    end
    pts(:, N+1) = obj_k;

    for i = 1:size(pts, 2)
        for j = 1:n_obs
            c_j = [obstacles(j).cx; obstacles(j).cy];
            r_j = obstacles(j).r + scp.collision_margin;
            dist_val = norm(pts(:,i) - c_j);
            if dist_val < r_j
                penetration = r_j - dist_val;
                J_coll = J_coll + scp.w_collision * penetration^4;
                n_collisions = n_collisions + 1;
            end
        end
    end
end

% ── Total cost ───────────────────────────────────────────────────────
J_total = J_pos + J_energy + J_coll;

% ── Pack info ────────────────────────────────────────────────────────
cost_info.J_total     = J_total;
cost_info.J_pos       = J_pos;
cost_info.J_energy    = J_energy;
cost_info.J_coll      = J_coll;
cost_info.x_land      = x_land;
cost_info.miss_m      = abs(x_land - p.task.d);
cost_info.miss_cm     = cost_info.miss_m * 100;
cost_info.n_collisions = n_collisions;

end