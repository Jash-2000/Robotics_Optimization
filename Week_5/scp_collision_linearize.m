function [A_coll, b_coll, info] = scp_collision_linearize(q_bar, obstacles, p, margin)
% scp_collision_linearize.m
% =========================================================================
% Linearize collision-avoidance constraints at reference configuration.
%
% For each check-point on the arm (link midpoints + object) and each
% obstacle, the nonlinear distance constraint
%
%   ||p(q) - c_j|| >= r_j + margin
%
% is replaced by a halfplane (first-order Taylor expansion):
%
%   n_hat' * [p(q_bar) + J_p * (q - q_bar)] >= r_j + margin
%
% where n_hat = (p(q_bar) - c_j) / ||p(q_bar) - c_j||.
%
% Returned in the form A_coll * q >= b_coll  (one row per active pair).
%
% INPUTS:
%   q_bar     — [4×1] reference joint angles
%   obstacles — obstacle struct array from load_obstacle_config
%   p         — parameter struct
%   margin    — safety margin [m] (default: 0.05)
%
% OUTPUTS:
%   A_coll — [n_rows × 4] constraint coefficient matrix
%   b_coll — [n_rows × 1] right-hand side
%   info   — struct with diagnostics
%
% CONVENTION:  The returned constraints are A_coll * q >= b_coll.
% To use inside quadprog (A*z <= b), negate:  -A_coll * q <= -b_coll.
% =========================================================================

if nargin < 4,  margin = 0.05;  end

N     = p.N;         % 4 joints
n_obs = numel(obstacles);
eps_fd = 1e-7;       % finite-difference step for point Jacobians

if n_obs == 0
    A_coll = zeros(0, N);
    b_coll = zeros(0, 1);
    info.n_constraints = 0;
    return;
end

% ── Compute reference forward kinematics ─────────────────────────────
[jpos_bar, ~, ee_bar] = forward_kinematics(q_bar, p);

% Object position at reference
alpha_N_bar = sum(q_bar);
R_N_bar = [cos(alpha_N_bar), -sin(alpha_N_bar);
           sin(alpha_N_bar),  cos(alpha_N_bar)];
obj_bar = ee_bar + R_N_bar * p.obj.r_gc;

% ── Build check-points and their Jacobians ───────────────────────────
% Check-points: link midpoints (4) + object CoM (1)
n_pts = N + 1;
pts = zeros(2, n_pts);
J_pts = zeros(2, N, n_pts);   % J_pts(:,:,i) = dp_i / dq  [2×4]

for i = 1:N
    pts(:, i) = 0.5 * (jpos_bar(:, i) + jpos_bar(:, i+1));
end
pts(:, N+1) = obj_bar;

% Compute Jacobians by finite differences on each check-point
for jj = 1:N
    q_p = q_bar;  q_p(jj) = q_p(jj) + eps_fd;
    q_m = q_bar;  q_m(jj) = q_m(jj) - eps_fd;

    [jpos_p, ~, ee_p] = forward_kinematics(q_p, p);
    [jpos_m, ~, ee_m] = forward_kinematics(q_m, p);

    alpha_p = sum(q_p);
    R_p = [cos(alpha_p), -sin(alpha_p); sin(alpha_p),  cos(alpha_p)];
    obj_p = ee_p + R_p * p.obj.r_gc;

    alpha_m = sum(q_m);
    R_m = [cos(alpha_m), -sin(alpha_m); sin(alpha_m),  cos(alpha_m)];
    obj_m = ee_m + R_m * p.obj.r_gc;

    for i = 1:N
        mid_p = 0.5 * (jpos_p(:, i) + jpos_p(:, i+1));
        mid_m = 0.5 * (jpos_m(:, i) + jpos_m(:, i+1));
        J_pts(:, jj, i) = (mid_p - mid_m) / (2*eps_fd);
    end
    J_pts(:, jj, N+1) = (obj_p - obj_m) / (2*eps_fd);
end

% ── Build halfplane constraints ──────────────────────────────────────
rows_A = [];
rows_b = [];

for i = 1:n_pts
    p_i   = pts(:, i);
    J_i   = J_pts(:, :, i);      % [2×4]

    for j = 1:n_obs
        c_j = [obstacles(j).cx; obstacles(j).cy];
        r_j = obstacles(j).r + margin;

        diff_vec = p_i - c_j;
        dist_val = norm(diff_vec);

        % Skip degenerate case (point exactly at obstacle centre)
        if dist_val < 1e-8
            continue;
        end

        n_hat = diff_vec / dist_val;

        % Linearized constraint:
        %   n_hat' * [p_i + J_i * (q - q_bar)] >= r_j
        %   n_hat' * J_i * q >= r_j - n_hat' * p_i + n_hat' * J_i * q_bar
        %   a_row * q >= b_val

        a_row = (n_hat' * J_i);     % [1×4]
        b_val = r_j - n_hat' * p_i + n_hat' * J_i * q_bar;

        rows_A = [rows_A; a_row];   %#ok<AGROW>
        rows_b = [rows_b; b_val];   %#ok<AGROW>
    end
end

A_coll = rows_A;
b_coll = rows_b;
info.n_constraints = size(A_coll, 1);

end