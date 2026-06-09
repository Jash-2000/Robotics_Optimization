function traj = unpack_collocation_solution(z_opt, nlp, p)
% unpack_collocation_solution.m
% =========================================================================
% Unpacks the optimized decision variable vector into interpretable
% trajectory data.
%
% INPUTS:
%   z_opt – optimal solution vector from IPOPT
%   nlp   – NLP structure
%   p     – parameter struct
%
% OUTPUT:
%   traj – struct with fields:
%     .t          -- time vector [M×1]
%     .q          -- joint angles [4×M]
%     .qdot       -- joint velocities [4×M]
%     .tau        -- joint torques [4×M]
%     .t_release  -- release time [scalar]
%     .x_land     -- landing x-coordinate [scalar]
%     .E_total    -- total energy consumed [scalar]
%     .obj_pos_release -- object position at release [2×1]
%     .obj_vel_release -- object velocity at release [2×1]
%
% =========================================================================

M = nlp.M;
N = nlp.N;

%% ── Unpack decision variable vector ──────────────────────────────────────
% z_opt is already numeric (extracted via sol.value in main_week3)
idx_q = 1:(N*M);
idx_qdot = (N*M+1):(2*N*M);
idx_tau = (2*N*M+1):(3*N*M);
idx_t = 3*N*M + 1;

traj.q = reshape(z_opt(idx_q), N, M);
traj.qdot = reshape(z_opt(idx_qdot), N, M);
traj.tau = reshape(z_opt(idx_tau), N, M);
traj.t_release = z_opt(idx_t);

% Time vector
traj.t = linspace(0, traj.t_release, M)';

%% ── Compute landing prediction ───────────────────────────────────────────
% Extract release state (last collocation point)
q_rel = traj.q(:, M);
qdot_rel = traj.qdot(:, M);

% Use Week 1 release_condition function
rc = release_condition(q_rel, qdot_rel, p);

traj.x_land = rc.x_land;
traj.obj_pos_release = rc.obj_pos;
traj.obj_vel_release = rc.obj_vel;

%% ── Compute total energy ─────────────────────────────────────────────────
dt = traj.t_release / M;
E_total = 0;

for k = 1:M
    power_k = traj.tau(:, k)' * traj.qdot(:, k);
    E_total = E_total + abs(power_k) * dt;
end

traj.E_total = E_total;

%% ── Additional info ──────────────────────────────────────────────────────
fprintf('\n');
fprintf('  Trajectory unpacked:\n');
fprintf('    Time points: %d\n', M);
fprintf('    Release time: %.2f s\n', traj.t_release);
fprintf('    Landing x: %.3f m\n', traj.x_land);
fprintf('    Total energy: %.2f J\n', traj.E_total);

end