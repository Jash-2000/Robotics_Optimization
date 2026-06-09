function [z_sol, qp_info] = scp_solve_subproblem(qp)
% scp_solve_subproblem.m
% =========================================================================
% Solve the convex QP sub-problem using MATLAB quadprog.
%
%   min  0.5 z'Hz + f'z
%   s.t. Aeq z = beq,   Aiq z <= biq,   lb <= z <= ub
%
% If the QP is infeasible, falls back to removing collision constraints
% (keeping dynamics, bounds, and slacks).
%
% INPUTS:
%   qp  — struct from scp_build_subproblem
%
% OUTPUTS:
%   z_sol   — [nz×1] optimal primal solution (or [] on hard failure)
%   qp_info — struct with: .exitflag, .fval, .output, .feasible
% =========================================================================

opts = optimoptions('quadprog', ...
    'Display',         'off', ...
    'MaxIterations',   2000, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance',   1e-12, ...
    'Algorithm',       'interior-point-convex');

% ── First attempt: full QP (dynamics + collision + slacks) ───────────
[z_sol, fval, exitflag, output] = quadprog( ...
    qp.H, qp.f, qp.Aiq, qp.biq, qp.Aeq, qp.beq, qp.lb, qp.ub, [], opts);

qp_info.exitflag = exitflag;
qp_info.fval     = fval;
qp_info.output   = output;
qp_info.feasible = (exitflag > 0);
qp_info.fallback = false;

if exitflag > 0
    return;   % success
end

% ── Fallback: drop collision constraints, keep dynamics ──────────────
fprintf('    ⚠  quadprog infeasible (exit %d); retrying without collision constraints...\n', exitflag);

[z_sol, fval, exitflag, output] = quadprog( ...
    qp.H, qp.f, [], [], qp.Aeq, qp.beq, qp.lb, qp.ub, [], opts);

qp_info.exitflag = exitflag;
qp_info.fval     = fval;
qp_info.output   = output;
qp_info.feasible = (exitflag > 0);
qp_info.fallback = true;

if exitflag <= 0
    fprintf('    ✗  quadprog failed even without collisions (exit %d)\n', exitflag);
    z_sol = [];
end

end