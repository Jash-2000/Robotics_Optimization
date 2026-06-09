% scp_diagnostic.m
% =========================================================================
% DEBUG SCRIPT: Check each SCP component step-by-step with detailed output
% =========================================================================

clear; close all; clc;

%% ── PATH SETUP ──────────────────────────────────────────────────────────
% Week 1 core functions (params, dynamics, kinematics, collision, etc.)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');
% Week 2 functions (shooting)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_2\');
% Week 3 functions (collocation + CasADi)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\');
% Week 4 functions (GCS)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_4\');

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║             SCP DIAGNOSTIC TEST — Component Inspection         ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% ── SETUP ──────────────────────────────────────────────────────────────
p = params();
p.opt.M = 60;

config_name = 'simple';
obstacles = load_obstacle_config(config_name, p);
p.obstacles = obstacles;
p.current_config = config_name;

scp.M = 60;
scp.trust_radius = 0.5;
scp.trust_radius_u = 5.0;
scp.t_release_min = 0.8;
scp.t_release_max = 2.5;
scp.w_position = 500;
scp.w_energy = 0.1;
scp.w_trust = 5.0;
scp.w_slack = 1e4;
scp.w_collision = 1e6;
scp.collision_margin = 0.05;

% ═════════════════════════════════════════════════════════════════════════
%% TEST 1: Initialize trajectory
% ═════════════════════════════════════════════════════════════════════════
fprintf('TEST 1: Initialize Trajectory (Standard)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    [x_bar, u_bar, t_f_bar] = scp_initialize_trajectory(p, scp, 'standard');
    fprintf('  ✓ Trajectory initialized\n');
    fprintf('    Shape: x [%d×%d], u [%d×%d]\n', size(x_bar,1), size(x_bar,2), size(u_bar,1), size(u_bar,2));
    fprintf('    t_f = %.2f s\n', t_f_bar);
    fprintf('    x range: q [%.3f, %.3f], qdot [%.3f, %.3f] rad/s\n', ...
        min(x_bar(1:4,:),[],'all'), max(x_bar(1:4,:),[],'all'), ...
        min(x_bar(5:8,:),[],'all'), max(x_bar(5:8,:),[],'all'));
    fprintf('    u range: [%.3f, %.3f] N·m\n', min(u_bar(:)), max(u_bar(:)));
    fprintf('    NaN checks: x=%d, u=%d, t=%d\n', ...
        nnz(isnan(x_bar)), nnz(isnan(u_bar)), isnan(t_f_bar));
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    rethrow(ME);
end

% ═════════════════════════════════════════════════════════════════════════
%% TEST 2: Evaluate initial cost
% ═════════════════════════════════════════════════════════════════════════
fprintf('\nTEST 2: Evaluate Initial Cost\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    [J_old, ci_old] = scp_evaluate_cost(x_bar, u_bar, t_f_bar, p, obstacles, scp);
    fprintf('  ✓ Cost evaluated\n');
    fprintf('    J_total = %.2f\n', J_old);
    fprintf('    J_pos   = %.2f  (landing: %.3f m, target: %.3f m, miss: %.1f cm)\n', ...
        ci_old.J_pos, ci_old.x_land, p.task.d, ci_old.miss_cm);
    fprintf('    J_energy = %.2f\n', ci_old.J_energy);
    fprintf('    J_coll   = %.2f  (n_collisions: %d)\n', ci_old.J_coll, ci_old.n_collisions);
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    rethrow(ME);
end

% ═════════════════════════════════════════════════════════════════════════
%% TEST 3: Build QP sub-problem
% ═════════════════════════════════════════════════════════════════════════
fprintf('\nTEST 3: Build QP Sub-problem (iteration 1)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    qp = scp_build_subproblem(x_bar, u_bar, t_f_bar, p, obstacles, scp);
    fprintf('  ✓ QP built successfully\n');
    fprintf('    Decision variables:\n');
    fprintf('      Core: 12*M + 1 = %d\n', qp.nz_core);
    fprintf('      Slacks: %d\n', qp.ns);
    fprintf('      Total: %d\n', qp.nz);
    fprintf('    Constraint matrices:\n');
    fprintf('      Aeq: [%d × %d] (dynamics + IC)\n', size(qp.Aeq,1), size(qp.Aeq,2));
    fprintf('      Aiq: [%d × %d] (collision + slacks)\n', size(qp.Aiq,1), size(qp.Aiq,2));
    fprintf('    Objective:\n');
    fprintf('      H sparsity: %.1f%%\n', 100*(1 - nnz(qp.H)/(qp.nz^2)));
    fprintf('      H eigenvalues (top 5): '); 
    ev = eigs(qp.H, 5, 'lm');
    fprintf('[%.2e, %.2e, %.2e, %.2e, %.2e]\n', ev(1), ev(2), ev(3), ev(4), ev(5));
    fprintf('      f norm: %.2e\n', norm(qp.f));
    fprintf('    Bounds at step 1:\n');
    ix1 = qp.idx_x(1);
    iu1 = qp.idx_u(1);
    var_names = {'q1','q2','q3','q4','qd1','qd2','qd3','qd4'};
    for jj = 1:8
        dev_lb = qp.lb(ix1(jj)) - x_bar(jj,1);
        dev_ub = qp.ub(ix1(jj)) - x_bar(jj,1);
        feasible_str = '';
        if qp.lb(ix1(jj)) > qp.ub(ix1(jj)) + 1e-6
            feasible_str = ' ← LB>UB INFEASIBLE!';
        end
        fprintf('      %4s: [%+.3f, %+.3f] around ref (raw lb=%.3f ub=%.3f)%s\n', ...
            var_names{jj}, dev_lb, dev_ub, qp.lb(ix1(jj)), qp.ub(ix1(jj)), feasible_str);
    end
    tau_names = {'t1','t2','t3','t4'};
    for jj = 1:4
        dev_lb = qp.lb(iu1(jj)) - u_bar(jj,1);
        dev_ub = qp.ub(iu1(jj)) - u_bar(jj,1);
        feasible_str = '';
        if qp.lb(iu1(jj)) > qp.ub(iu1(jj)) + 1e-6
            feasible_str = ' ← LB>UB INFEASIBLE!';
        end
        fprintf('      %4s: [%+.3f, %+.3f] around ref%s\n', ...
            tau_names{jj}, dev_lb, dev_ub, feasible_str);
    end
    fprintf('      (t_f is no longer a QP variable — updated separately)\n');
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    fprintf('     Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('       %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end

% ═════════════════════════════════════════════════════════════════════════
%% TEST 3.5: INSPECT QP MATRIX STRUCTURE (NEW)
% ═════════════════════════════════════════════════════════════════════════
fprintf('\nTEST 3.5: Inspect QP Matrix Structure\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

fprintf('  H matrix analysis:\n');
fprintf('    Shape: [%d × %d]\n', size(qp.H,1), size(qp.H,2));
fprintf('    Sparsity: %.1f%% (%.0f non-zeros)\n', ...
    100*(1 - nnz(qp.H)/(qp.nz^2)), nnz(qp.H));
fprintf('    Symmetric: %s\n', mat2str(issymmetric(qp.H)));

% Check if H is positive semi-definite
H_full = full(qp.H);
try
    chol(H_full);
    fprintf('    Positive definite: YES (Cholesky succeeded)\n');
catch
    fprintf('    Positive definite: NO (Cholesky failed)\n');
    % Use eig on a small block to avoid singularity crash
    block_size = min(50, qp.nz);
    ev_block = eig(H_full(1:block_size, 1:block_size));
    ev_block = sort(real(ev_block));
    fprintf('    Smallest eigenvalues (top-%d block): [%.2e, %.2e, %.2e]\n', ...
        block_size, ev_block(1), ev_block(2), ev_block(3));
    fprintf('    Negative eigenvalues in block: %d\n', nnz(ev_block < -1e-8));
end
fprintf('    H diagonal range: [%.2e, %.2e]\n', min(diag(H_full)), max(diag(H_full)));
n_zero_diag = nnz(abs(diag(H_full)) < 1e-10);
fprintf('    H diagonal zeros: %d / %d\n', n_zero_diag, qp.nz);
if n_zero_diag > 0
    fprintf('    ✗ %d zero diagonal entries — unbounded direction!\n', n_zero_diag);
else
    fprintf('    ✓ All diagonal entries non-zero\n');
end

fprintf('\n  Aeq matrix analysis:\n');
fprintf('    Shape: [%d × %d]\n', size(qp.Aeq,1), size(qp.Aeq,2));
fprintf('    Rank: %d (full rank: %s)\n', rank(full(qp.Aeq)), ...
    mat2str(rank(full(qp.Aeq)) == size(qp.Aeq,1)));
fprintf('    Norm estimate: %.2e\n', normest(qp.Aeq));

fprintf('\n  Bounds analysis:\n');
fprintf('    lb range: [%.3f, %.3f]\n', min(qp.lb), max(qp.lb));
fprintf('    ub range: [%.3f, %.3f]\n', min(qp.ub), max(qp.ub));
fprintf('    Feasible bounds: %s (lb <= ub)\n', ...
    mat2str(all(qp.lb <= qp.ub + 1e-10)));
n_infeasible_bounds = nnz(qp.lb > qp.ub + 1e-6);
if n_infeasible_bounds > 0
    fprintf('    ✗ %d variables have lb > ub (INFEASIBLE!)\n', n_infeasible_bounds);
end

fprintf('\n  f vector analysis:\n');
fprintf('    Norm: %.2e\n', norm(qp.f));
fprintf('    Range: [%.2e, %.2e]\n', min(qp.f), max(qp.f));

fprintf('\n  Aiq matrix analysis:\n');
fprintf('    Shape: [%d × %d]\n', size(qp.Aiq,1), size(qp.Aiq,2));
fprintf('    Non-zeros: %.0f\n', nnz(qp.Aiq));
fprintf('    Sparsity: %.1f%%\n', 100*(1 - nnz(qp.Aiq)/(size(qp.Aiq,1)*size(qp.Aiq,2))));

fprintf('\n  beq/biq analysis:\n');
fprintf('    beq: norm = %.2e, range = [%.2e, %.2e]\n', ...
    norm(qp.beq), min(qp.beq), max(qp.beq));
fprintf('    biq: norm = %.2e, range = [%.2e, %.2e]\n', ...
    norm(qp.biq), min(qp.biq), max(qp.biq));

% Check constraint compatibility with reference trajectory
fprintf('\n  Constraint feasibility check:\n');
z_test = zeros(qp.nz, 1);
for k = 1:qp.M
    z_test(qp.idx_x(k)) = x_bar(:, k);
    z_test(qp.idx_u(k)) = u_bar(:, k);
end
% Fill slacks with 0 (worst case — slacks not helping)
if qp.ns > 0
    z_test(qp.nz_core+1:end) = 0;
end

eq_violation = norm(qp.Aeq * z_test - qp.beq);
iq_violation = max(0, max(qp.Aiq * z_test - qp.biq));
fprintf('    Reference trajectory (x_bar, u_bar):\n');
fprintf('      Eq constraint violation: %.2e\n', eq_violation);
fprintf('      Ineq constraint violation: %.2e\n', iq_violation);
fprintf('      Bounds violated: %d vars\n', ...
    nnz((z_test < qp.lb - 1e-6) | (z_test > qp.ub + 1e-6)));
fprintf('\nTEST 4: Solve QP with quadprog\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

    try
    [z_sol, qp_info] = scp_solve_subproblem(qp);
    if isempty(z_sol)
        fprintf('  ✗ quadprog returned empty (exitflag: %d)\n', qp_info.exitflag);
        fprintf('    Fallback used: %s\n', mat2str(qp_info.fallback));
        fprintf('    (This happens when the QP is infeasible)\n');
    else
        fprintf('  ✓ QP solved\n');
        exitflag_messages = {'Optimal'; 'Unbounded'; 'Infeas'; 'Error'; 'Iter limit'};
        ef = min(qp_info.exitflag + 2, length(exitflag_messages));
        ef = max(ef, 1);
        fprintf('    Exitflag: %d  (%s)\n', qp_info.exitflag, exitflag_messages{ef});
        fprintf('    Objective value: %.2e\n', qp_info.fval);
        fprintf('    Output: %s\n', qp_info.output.message);
        fprintf('    Fallback used: %s\n', mat2str(qp_info.fallback));
        
        % Check for NaN in solution
        n_nan = nnz(isnan(z_sol));
        fprintf('    Solution NaN count: %d / %d\n', n_nan, length(z_sol));
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    rethrow(ME);
end

% ═════════════════════════════════════════════════════════════════════════
%% TEST 5: Unpack solution
% ═════════════════════════════════════════════════════════════════════════
fprintf('\nTEST 5: Unpack Solution\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

if ~isempty(z_sol)
    try
        [x_new, u_new] = scp_unpack_solution(z_sol, qp);
        fprintf('  ✓ Solution unpacked\n');
        fprintf('    x_new: [%d × %d] NaN: %d\n', size(x_new,1), size(x_new,2), nnz(isnan(x_new)));
        fprintf('    u_new: [%d × %d] NaN: %d\n', size(u_new,1), size(u_new,2), nnz(isnan(u_new)));
        fprintf('    State change: max ||x_new - x_bar|| = %.4f\n', max(abs(x_new(:) - x_bar(:))));
        t_f_new = scp_update_release_time(x_new, u_new, t_f_bar, p, obstacles, scp);
        fprintf('    t_f updated (1-D search): %.2f s  (was %.2f s)\n', t_f_new, t_f_bar);
    catch ME
        fprintf('  ✗ FAILED: %s\n', ME.message);
        rethrow(ME);
    end
else
    fprintf('  (Skipped — QP solution is empty)\n');
end

% ═════════════════════════════════════════════════════════════════════════
%% TEST 6: Evaluate new cost & trust ratio
% ═════════════════════════════════════════════════════════════════════════
fprintf('\nTEST 6: Evaluate New Cost & Trust Ratio\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

if ~isempty(z_sol)
    try
        [J_new, ci_new] = scp_evaluate_cost(x_new, u_new, t_f_new, p, obstacles, scp);

        % Correct J_pred: model cost at x_new (no trust penalty / QP offset)
        x_land_pred = qp.x_land_bar + qp.grad_x_land' * (x_new(:,scp.M) - x_bar(:,scp.M));
        J_energy_new = 0;
        for kk = 1:scp.M
            J_energy_new = J_energy_new + scp.w_energy*(u_new(:,kk)'*u_new(:,kk))*qp.h;
        end
        J_pred_model = scp.w_position * (x_land_pred - p.task.d)^2 + J_energy_new;

        fprintf('  ✓ Cost evaluated\n');
        fprintf('    J_old  (actual at ref):        %.4f\n', J_old);
        fprintf('    J_new  (actual at candidate):  %.4f\n', J_new);
        fprintf('    J_pred (raw QP fval, ignore):  %.2e\n', qp_info.fval);
        fprintf('    J_pred_model (correct, use):   %.4f\n', J_pred_model);
        fprintf('    x_land predicted: %.3f m\n', x_land_pred);
        fprintf('    x_land actual:    %.3f m\n', ci_new.x_land);

        if abs(J_old - J_pred_model) > 1e-12
            xi = (J_old - J_new) / (J_old - J_pred_model);
            fprintf('    Trust ratio ξ = %.4f\n', xi);
            if xi > 0.75,      fprintf('      → Accept + Expand  ✓\n');
            elseif xi > 0.1,   fprintf('      → Accept  ✓\n');
            elseif xi > 0,     fprintf('      → Accept + Contract\n');
            else,              fprintf('      → REJECT + Contract  ✗ (step worsens cost)\n'); end
        end

        fprintf('    New miss: %.2f cm  (was: %.2f cm)\n', ci_new.miss_cm, ci_old.miss_cm);
        fprintf('    New collisions: %d\n', ci_new.n_collisions);
    catch ME
        fprintf('  ✗ FAILED: %s\n', ME.message);
        rethrow(ME);
    end
else
    fprintf('  (Skipped — QP solution is empty)\n');
end

% ═════════════════════════════════════════════════════════════════════════
fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    DIAGNOSTICS COMPLETE                       ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('SUMMARY:\n');
fprintf('  If all tests pass, the QP formulation is correct.\n');
fprintf('  If the QP solution is empty, the problem is infeasible.\n');
fprintf('  If J_new is much worse than J_old, the trust region is too large.\n');
fprintf('  If ξ is very close to 0, the model is not predicting the cost well.\n\n');