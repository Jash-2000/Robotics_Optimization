function [A, B] = scp_dynamics_jacobians(x, u, p)
% scp_dynamics_jacobians.m
% =========================================================================
% Compute dynamics Jacobians via central finite differences.
%
%   f(x, u) = [qdot; M(q)\(u - C(q,qdot)*qdot - G(q))]
%
%   A = df/dx  evaluated at (x, u)   — [8×8]
%   B = df/du  evaluated at (x, u)   — [8×4]
%
% Central differences are O(eps^2) accurate.  For M=60, the per-iteration
% cost is 24 function evaluations × 60 steps = 1440 evaluations, which is
% acceptable for a research comparison.
%
% NOTE: B has a known analytic structure because f is affine in u:
%   B = [0_{4×4}; M(q)^{-1}]
% We verify this via FD anyway for safety but could exploit it for speed.
%
% INPUTS:
%   x  — [8×1] state  [q; qdot]
%   u  — [4×1] torque
%   p  — parameter struct
%
% OUTPUTS:
%   A  — [8×8] state Jacobian
%   B  — [8×4] control Jacobian
% =========================================================================

nx = 8;   % state dimension
nu = 4;   % control dimension
eps_x = 1e-6;
eps_u = 1e-6;

% ── State Jacobian A = df/dx ────────────────────────────────────────────
A = zeros(nx, nx);
for j = 1:nx
    xp = x;  xp(j) = xp(j) + eps_x;
    xm = x;  xm(j) = xm(j) - eps_x;
    A(:, j) = (arm_dynamics_continuous(xp, u, p) ...
             - arm_dynamics_continuous(xm, u, p)) / (2*eps_x);
end

% ── Control Jacobian B = df/du ──────────────────────────────────────────
B = zeros(nx, nu);
for j = 1:nu
    up = u;  up(j) = up(j) + eps_u;
    um = u;  um(j) = um(j) - eps_u;
    B(:, j) = (arm_dynamics_continuous(x, up, p) ...
             - arm_dynamics_continuous(x, um, p)) / (2*eps_u);
end

end