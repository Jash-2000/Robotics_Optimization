function qp = scp_build_subproblem(x_bar, u_bar, t_f_bar, p, obstacles, scp)
% scp_build_subproblem.m
% =========================================================================
% Build the convex QP sub-problem for one SCP iteration.
%
% Decision variables:  z = [x_1; u_1; x_2; u_2; ...; x_M; u_M; s_1; ...; s_Ns]
%
%   NOTE: t_f is NOT a decision variable here. h = t_f_bar/M is fixed
%   for this sub-problem. t_f is updated separately via a 1-D line search
%   in scp_update_release_time.m after each SCP step. This keeps the QP
%   well-conditioned and avoids an unconnected free variable.
%
%   Core vars:  nz_core = (nx + nu) * M  = 12 * 60 = 720
%   Slacks:     ns      = total collision constraint rows
%   Total:      nz      = nz_core + ns
%
% QP form:  min  0.5 z'Hz + f'z
%           s.t. Aeq z  = beq        (linearised forward-Euler dynamics + IC)
%                Aiq z <= biq        (collision half-planes with slack)
%                lb  <= z <= ub      (trust region + physical limits)
%
% INPUTS:
%   x_bar   — [8×M]   reference state trajectory
%   u_bar   — [4×M]   reference control trajectory
%   t_f_bar — scalar   current release time (fixed for this QP)
%   p       — parameter struct
%   obstacles — obstacle struct array
%   scp     — SCP parameter struct
%
% OUTPUT:
%   qp — struct passed directly to scp_solve_subproblem
% =========================================================================

M  = scp.M;
N  = p.N;      % 4 joints
nx = 2*N;      % 8 states
nu = N;        % 4 controls
h  = t_f_bar / M;          % fixed time step

% ── Index helpers ────────────────────────────────────────────────────────
% Layout per step k:  [x_k (8 vars) | u_k (4 vars)]
nz_core = (nx + nu) * M;   % 720
idx_x = @(k) (k-1)*(nx+nu) + (1:nx);
idx_u = @(k) (k-1)*(nx+nu) + nx + (1:nu);

% ==========================================================================
%  PASS 1 — count collision constraints (pre-allocate slacks)
% ==========================================================================
coll_data = cell(M, 1);
ns_total  = 0;
for k = 1:M
    q_bar_k = x_bar(1:N, k);
    [Ac_k, bc_k] = scp_collision_linearize(q_bar_k, obstacles, p, scp.collision_margin);
    coll_data{k}.Ac = Ac_k;
    coll_data{k}.bc = bc_k;
    ns_total = ns_total + size(Ac_k, 1);
end

nz = nz_core + ns_total;

% ==========================================================================
%  EQUALITY CONSTRAINTS  (linearised dynamics around reference + IC)
%
%  The QP enforces the Forward-Euler linearisation of f around (x_bar, u_bar):
%
%   x_{k+1} = x_k + h*[ f_k + A_k*(x_k - x_bar_k) + B_k*(u_k - u_bar_k) ]
%
%  Rearranging:
%   x_{k+1} - (I + h*A_k)*x_k - h*B_k*u_k = beq_k
%
%  where beq_k is chosen so (x_bar, u_bar) exactly satisfies this:
%   beq_k = x_bar_{k+1} - (I + h*A_k)*x_bar_k - h*B_k*u_bar_k
%
%  This is exact for ANY integrator used to generate x_bar (RK4, ODE45, etc.)
%  and ensures the reference trajectory is always in the QP feasible set.
% ==========================================================================
n_eq = nx + nx*(M-1);
Aeq  = zeros(n_eq, nz);
beq  = zeros(n_eq, 1);

% Initial condition: x_1 = [q0; qdot0]
Aeq(1:nx, idx_x(1)) = eye(nx);
beq(1:nx) = [p.q0; p.qdot0];

% Linearised dynamics for k = 1 ... M-1
for k = 1:M-1
    [A_k, B_k] = scp_dynamics_jacobians(x_bar(:,k), u_bar(:,k), p);

    row = nx + (k-1)*nx + (1:nx);
    Aeq(row, idx_x(k+1)) =  eye(nx);
    Aeq(row, idx_x(k))   = -(eye(nx) + h*A_k);
    Aeq(row, idx_u(k))   = -h*B_k;

    % Exact RHS: ensures x_bar satisfies Aeq*z_bar = beq exactly
    beq(row) = x_bar(:,k+1) - (eye(nx) + h*A_k)*x_bar(:,k) - h*B_k*u_bar(:,k);
end

Aeq = sparse(Aeq);

% ==========================================================================
%  INEQUALITY CONSTRAINTS  (collision half-planes + slack variables)
%
%  Constraint per row j at step k:   a_j' * q_k >= b_j
%  With slack s_j >= 0:              a_j' * q_k + s_j >= b_j
%  In QP (<= form):                 -a_j' * q_k - s_j <= -b_j
% ==========================================================================
Aiq = zeros(ns_total, nz);
biq = zeros(ns_total, 1);

row_ptr   = 0;
slack_ptr = nz_core;   % slacks start after core variables

for k = 1:M
    Ac_k = coll_data{k}.Ac;
    bc_k = coll_data{k}.bc;
    nr = size(Ac_k, 1);
    if nr == 0, continue; end

    ix_k = idx_x(k);
    iq_k = ix_k(1:N);    % only q entries (first N of the 8 state vars)

    for rr = 1:nr
        row_ptr   = row_ptr  + 1;
        slack_ptr = slack_ptr + 1;
        Aiq(row_ptr, iq_k)     = -Ac_k(rr, :);
        Aiq(row_ptr, slack_ptr) = -1;
        biq(row_ptr) = -bc_k(rr);
    end
end

Aiq = sparse(Aiq);

% ==========================================================================
%  BOUNDS  (trust region + physical limits + slack non-negativity)
% ==========================================================================
lb = -inf(nz, 1);
ub =  inf(nz, 1);

rho   = scp.trust_radius;
rho_u = scp.trust_radius_u;

for k = 1:M
    ix_k  = idx_x(k);
    iq_k  = ix_k(1:N);       % joint angle indices
    iqd_k = ix_k(N+1:end);   % joint velocity indices
    iu_k  = idx_u(k);

    % ── Joint positions: trust region (no angle-limit clipping here —
    %    the initial trajectory may already violate q_min/q_max, and we
    %    need the QP to be feasible around the reference) ───────────────
    lb(iq_k)  = x_bar(1:N, k)     - rho;
    ub(iq_k)  = x_bar(1:N, k)     + rho;

    % ── Joint velocities: trust region ONLY — do NOT clip to qd limits.
    %    The initialisation produces qdot up to ±42 rad/s.  Clipping to
    %    ±5 rad/s creates lb > ub when qdot_bar is outside that range,
    %    making the QP immediately infeasible. ────────────────────────────
    lb(iqd_k) = x_bar(N+1:end, k) - rho;
    ub(iqd_k) = x_bar(N+1:end, k) + rho;

    % ── Torques: trust region clipped to actuator limits ─────────────────
    lb(iu_k)  = max(u_bar(:, k) - rho_u, p.lim.tau_min);
    ub(iu_k)  = min(u_bar(:, k) + rho_u, p.lim.tau_max);

    % ── Safety: ensure lb <= ub (guards against tau limit conflicts) ──────
    lb(iu_k) = min(lb(iu_k), ub(iu_k) - 1e-8);
end

% Slacks: non-negative
if ns_total > 0
    lb(nz_core+1:nz) = 0;
    ub(nz_core+1:nz) = inf;
end

% ==========================================================================
%  OBJECTIVE  0.5 z'Hz + f'z
%
%  1. Energy proxy:     w_E * sum_k ||u_k||^2 * h
%  2. Trust penalty:    w_rho * sum_k ||x_k - x_bar_k||^2
%  3. Position cost:    w_pos * (x_land_linearised - d)^2
%  4. Slack penalty:    w_s * sum_j s_j^2
% ==========================================================================
H     = sparse(nz, nz);
f_obj = zeros(nz, 1);

w_energy = scp.w_energy;
w_trust  = scp.w_trust;
w_pos    = scp.w_position;
w_slack  = scp.w_slack;

% ── 1. Energy ────────────────────────────────────────────────────────────
for k = 1:M
    iu_k = idx_u(k);
    for j = 1:nu
        H(iu_k(j), iu_k(j)) = H(iu_k(j), iu_k(j)) + 2*w_energy*h;
    end
end

% ── 2. Trust-region penalty ───────────────────────────────────────────────
for k = 1:M
    ix_k = idx_x(k);
    for j = 1:nx
        H(ix_k(j), ix_k(j)) = H(ix_k(j), ix_k(j)) + 2*w_trust;
        f_obj(ix_k(j))       = f_obj(ix_k(j))       - 2*w_trust * x_bar(j, k);
    end
end

% ── 3. Position cost (linearised x_land around x_M_bar) ──────────────────
x_M_bar = x_bar(:, M);
rc_bar  = release_condition(x_M_bar(1:N), x_M_bar(N+1:end), p);
x_land_bar = rc_bar.x_land;

if isnan(x_land_bar)
    fprintf('    ⚠  release_condition returned NaN at x_bar_M — skipping position cost\n');
    x_land_bar = p.task.d;   % fallback: pretend we're on target
end

% Finite-difference gradient d(x_land)/d(x_M)
grad = zeros(nx, 1);
eps_g = 1e-6;
for j = 1:nx
    xp = x_M_bar; xp(j) = xp(j) + eps_g;
    xm = x_M_bar; xm(j) = xm(j) - eps_g;
    rc_p = release_condition(xp(1:N), xp(N+1:end), p);
    rc_m = release_condition(xm(1:N), xm(N+1:end), p);
    xl_p = rc_p.x_land;  if isnan(xl_p), xl_p = x_land_bar; end
    xl_m = rc_m.x_land;  if isnan(xl_m), xl_m = x_land_bar; end
    grad(j) = (xl_p - xl_m) / (2*eps_g);
end

% x_land ≈ x_land_bar + grad'*(x_M - x_M_bar)
% cost = w_pos*(x_land - d)^2,  let  err = x_land_bar - d
err = x_land_bar - p.task.d;

% QP contribution:  w_pos*(err + grad'*(x_M - x_M_bar))^2
%   = w_pos*(err - grad'*x_M_bar)^2 + 2*w_pos*(err - grad'*x_M_bar)*grad'*x_M
%                                    + w_pos*(x_M'*grad*grad'*x_M)
% H term:  2*w_pos * grad*grad'  (added to ix_M block)
% f term:  2*w_pos * (err - grad'*x_M_bar) * grad  (but simpler: 2*w_pos*err*grad)
% [note: x_M_bar contribution is a constant, doesn't affect optimisation]

ix_M = idx_x(M);
g_outer = grad * grad';  % [nx × nx]
for r = 1:nx
    for c = 1:nx
        if abs(g_outer(r,c)) > 1e-15
            H(ix_M(r), ix_M(c)) = H(ix_M(r), ix_M(c)) + 2*w_pos * g_outer(r,c);
        end
    end
end
for j = 1:nx
    f_obj(ix_M(j)) = f_obj(ix_M(j)) + 2*w_pos * err * grad(j);
end

% ── 4. Slack penalty ─────────────────────────────────────────────────────
for jj = 1:ns_total
    H(nz_core + jj, nz_core + jj) = H(nz_core + jj, nz_core + jj) + 2*w_slack;
end

% ── Symmetrise + regularise H ────────────────────────────────────────────
H = (H + H') * 0.5;
H = H + 1e-6 * speye(nz);   % small ridge to ensure strict positive-definiteness

% ==========================================================================
%  PACKAGE OUTPUT
% ==========================================================================
qp.H        = H;
qp.f        = f_obj;
qp.Aeq      = Aeq;
qp.beq      = beq;
qp.Aiq      = Aiq;
qp.biq      = biq;
qp.lb       = lb;
qp.ub       = ub;
qp.nz       = nz;
qp.nz_core  = nz_core;
qp.ns       = ns_total;
qp.M        = M;
qp.N        = N;
qp.nx       = nx;
qp.nu       = nu;
qp.h        = h;
qp.idx_x    = idx_x;
qp.idx_u    = idx_u;
% NOTE: no idx_tf — t_f is no longer a QP decision variable
qp.x_land_bar   = x_land_bar;
qp.grad_x_land  = grad;

end