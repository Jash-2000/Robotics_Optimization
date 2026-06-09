function [t_f_new, J_new_tf] = scp_update_release_time(x_new, u_new, t_f_bar, p, obstacles, scp)
% scp_update_release_time.m
% =========================================================================
% Update the release time t_f via a 1-D golden-section search.
%
% Given a fixed trajectory shape (x, u), find the release time t_f in
% [t_release_min, t_release_max] that minimises the true nonlinear cost.
%
% This is called AFTER each QP solve instead of including t_f in the QP,
% which keeps the QP well-conditioned.
%
% INPUTS:
%   x_new   — [8×M] new state trajectory from QP
%   u_new   — [4×M] new control trajectory from QP
%   t_f_bar — current release time (starting point)
%   p, obstacles, scp — as usual
%
% OUTPUTS:
%   t_f_new  — updated release time
%   J_new_tf — cost at new t_f
% =========================================================================

% Golden section search over [t_min, t_max]
t_lo = scp.t_release_min;
t_hi = scp.t_release_max;
tol  = 1e-3;   % 1 ms precision is sufficient
gr   = (sqrt(5) - 1) / 2;   % golden ratio

a = t_lo;
b = t_hi;
c = b - gr*(b - a);
d = a + gr*(b - a);

f_c = cost_at_tf(x_new, u_new, c, p, obstacles, scp);
f_d = cost_at_tf(x_new, u_new, d, p, obstacles, scp);

n_eval = 0;
max_eval = 30;

while abs(b - a) > tol && n_eval < max_eval
    if f_c < f_d
        b = d;
        d = c;  f_d = f_c;
        c = b - gr*(b - a);
        f_c = cost_at_tf(x_new, u_new, c, p, obstacles, scp);
    else
        a = c;
        c = d;  f_c = f_d;
        d = a + gr*(b - a);
        f_d = cost_at_tf(x_new, u_new, d, p, obstacles, scp);
    end
    n_eval = n_eval + 1;
end

t_f_new  = (a + b) / 2;
J_new_tf = cost_at_tf(x_new, u_new, t_f_new, p, obstacles, scp);

end

% ── Helper: evaluate cost at a given t_f ─────────────────────────────────
function J = cost_at_tf(x_traj, u_traj, t_f, p, obstacles, scp)
    [J, ~] = scp_evaluate_cost(x_traj, u_traj, t_f, p, obstacles, scp);
end