function [results, scp_log] = run_method_M5_scp(config_name, p)
% run_method_M5_scp.m  — M5: Standard SCP with backtracking line search
% =========================================================================

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  M5: STANDARD SCP  —  %s\n', upper(config_name));
fprintf('═══════════════════════════════════════════════════════════════\n');

obstacles = load_obstacle_config(config_name, p);
p.obstacles   = obstacles;
p.current_config = config_name;

%% ── SCP parameters ───────────────────────────────────────────────────────
scp.M               = 60;
scp.max_iters       = 50;
scp.convergence_tol = 1e-3;

scp.trust_radius       = 0.5;
scp.trust_radius_u     = 5.0;
scp.trust_radius_min   = 1e-4;
scp.trust_radius_max   = 5.0;
scp.trust_radius_u_min = 0.1;
scp.trust_radius_u_max = 20.0;

scp.t_release_min = 0.8;
scp.t_release_max = 2.5;

scp.w_position  = 200;
scp.w_energy    = 0.1;
scp.w_trust     = 5.0;
scp.w_slack     = 1e4;
scp.w_collision = 1e7;
scp.collision_margin = 0.2;

fprintf('  M=%d, max_iters=%d, rho=%.2f, rho_u=%.1f\n', ...
    scp.M, scp.max_iters, scp.trust_radius, scp.trust_radius_u);

%% ── Initialise ───────────────────────────────────────────────────────────
fprintf('\n  Initialising...\n');
try
    [x_bar, u_bar, t_f_bar] = scp_initialize_trajectory(p, scp, 'standard');
catch ME
    fprintf('  ✗ Init failed: %s\n', ME.message);
    results = make_empty_result('M5 (Standard SCP)', config_name, ME.message);
    scp_log = struct([]);
    return;
end

[J_old, ci_old] = scp_evaluate_cost(x_bar, u_bar, t_f_bar, p, obstacles, scp);
J_best = J_old;
x_best = x_bar; u_best = u_bar; t_f_best = t_f_bar;

fprintf('  Initial: miss=%.1f cm, J=%.1f, collisions=%d, t_f=%.2f\n', ...
    ci_old.miss_cm, J_old, ci_old.n_collisions, t_f_bar);

%% ── SCP loop ─────────────────────────────────────────────────────────────
scp_log = struct([]);
n_reject = 0;
total_tic = tic;

for iter = 1:scp.max_iters
    iter_tic = tic;

    % Build QP
    try
        qp = scp_build_subproblem(x_bar, u_bar, t_f_bar, p, obstacles, scp);
    catch ME
        fprintf('  iter %2d: BUILD FAILED — %s\n', iter, ME.message);
        break;
    end

    % Solve QP
    [z_sol, qp_info] = scp_solve_subproblem(qp);
    if isempty(z_sol)
        fprintf('  iter %2d: QP infeasible — contracting\n', iter);
        scp.trust_radius   = max(scp.trust_radius   * 0.5, scp.trust_radius_min);
        scp.trust_radius_u = max(scp.trust_radius_u * 0.5, scp.trust_radius_u_min);
        n_reject = n_reject + 1;
        if n_reject > 10
            fprintf('  ⚠  Too many consecutive rejects\n');
            break;
        end
        continue;
    end

    % Unpack full QP step
    [x_full, u_full] = scp_unpack_solution(z_sol, qp);

    % ── Backtracking line search ──────────────────────────────────────
    % Try full step first, then 0.5, 0.25, 0.125
    alphas = [1.0, 0.5, 0.25, 0.125];
    accepted_alpha = 0;
    J_new = inf;

    for ai = 1:length(alphas)
        alpha = alphas(ai);
        x_try = x_bar + alpha * (x_full - x_bar);
        u_try = u_bar + alpha * (u_full - u_bar);
        t_f_try = scp_update_release_time(x_try, u_try, t_f_bar, p, obstacles, scp);
        [J_try, ci_try] = scp_evaluate_cost(x_try, u_try, t_f_try, p, obstacles, scp);

        if J_try < J_old - 1e-10
            % Cost decreased — accept this step size
            x_new   = x_try;
            u_new   = u_try;
            t_f_new = t_f_try;
            J_new   = J_try;
            ci_new  = ci_try;
            accepted_alpha = alpha;
            break;
        end
    end

    dx_max  = max(abs(x_full(:) - x_bar(:)));
    qp_time = toc(iter_tic);

    if accepted_alpha > 0
        % Step accepted (with possible line-search reduction)
        n_reject = 0;

        % Trust region update based on agreement
        x_land_pred = qp.x_land_bar + qp.grad_x_land' * (x_new(:,scp.M) - x_bar(:,scp.M));
        J_energy_new = 0;
        for kk = 1:scp.M
            J_energy_new = J_energy_new + scp.w_energy * (u_new(:,kk)'*u_new(:,kk)) * qp.h;
        end
        J_pred = scp.w_position * (x_land_pred - p.task.d)^2 + J_energy_new;

        [~, rho_new, rho_u_new, tr_info] = ...
            scp_trust_region_update(J_old, J_new, J_pred, ...
            scp.trust_radius, scp.trust_radius_u, scp);

        if accepted_alpha < 1
            % Had to reduce step — contract trust region
            rho_new   = max(scp.trust_radius * 0.5, scp.trust_radius_min);
            rho_u_new = max(scp.trust_radius_u * 0.5, scp.trust_radius_u_min);
            action_str = sprintf('accept α=%.2f + contract', accepted_alpha);
        else
            action_str = tr_info.action;
        end

        % Update reference
        x_bar   = x_new;
        u_bar   = u_new;
        t_f_bar = t_f_new;
        J_old   = J_new;

        % Track best solution
        if J_new < J_best
            J_best = J_new; x_best = x_new; u_best = u_new; t_f_best = t_f_new;
        end

        scp.trust_radius   = rho_new;
        scp.trust_radius_u = rho_u_new;

        fprintf('  iter %2d | miss=%6.1f cm | J=%7.2f | dx=%.4f | rho=%.3f | [%s] | %.1fs\n', ...
            iter, ci_new.miss_cm, J_new, dx_max, scp.trust_radius, action_str, qp_time);
    else
        % All line-search steps failed — reject and contract
        n_reject = n_reject + 1;
        scp.trust_radius   = max(scp.trust_radius   * 0.5, scp.trust_radius_min);
        scp.trust_radius_u = max(scp.trust_radius_u * 0.5, scp.trust_radius_u_min);

        fprintf('  iter %2d | miss=%6.1f cm | J=%7.2f | dx=%.4f | rho=%.3f | [REJECT all α] | %.1fs\n', ...
            iter, ci_old.miss_cm, J_old, dx_max, scp.trust_radius, qp_time);

        if n_reject > 10
            fprintf('  ⚠  Too many consecutive rejects — stopping\n');
            break;
        end
    end

    % Log
    entry.iter     = iter;
    entry.J_total  = J_old;
    entry.miss_cm  = ci_old.miss_cm;
    entry.dx_max   = dx_max;
    entry.rho      = scp.trust_radius;
    entry.accepted = (accepted_alpha > 0);
    entry.alpha    = accepted_alpha;
    entry.qp_time  = qp_time;
    if isempty(scp_log); scp_log = entry; else; scp_log(end+1) = entry; end

    % Convergence check
    if accepted_alpha > 0 && dx_max < scp.convergence_tol
        fprintf('  ✓ Converged (iter %d, dx=%.2e)\n', iter, dx_max);
        break;
    end
    if scp.trust_radius <= scp.trust_radius_min && n_reject > 3
        fprintf('  ⚠  Trust region collapsed\n');
        break;
    end
end

t_total = toc(total_tic);

%% ── Use BEST solution found ─────────────────────────────────────────────
x_bar = x_best; u_bar = u_best; t_f_bar = t_f_best;

%% ── Post-process ────────────────────────────────────────────────────────
[~, ci_final] = scp_evaluate_cost(x_bar, u_bar, t_f_bar, p, obstacles, scp);
[n_arm, n_flt] = count_collisions(x_bar, t_f_bar, obstacles, p, scp);

N = p.N;  h = t_f_bar / scp.M;
E_total = 0;
for k = 1:scp.M
    E_total = E_total + abs(u_bar(:,k)' * x_bar(N+1:end,k)) * h;
end

results.method        = 'M5 (Standard SCP)';
results.config        = config_name;
results.miss_distance = ci_final.miss_cm;
results.x_land        = ci_final.x_land;
results.t_total       = t_total;
results.iterations    = numel(scp_log);
results.energy        = E_total;
results.t_release     = t_f_bar;
results.n_collisions  = n_arm + n_flt;
results.n_arm_coll    = n_arm;
results.n_flight_coll = n_flt;
results.collision_free = (n_arm + n_flt == 0);
results.trajectory.x  = x_bar;
results.trajectory.u  = u_bar;
results.trajectory.t_f = t_f_bar;

fprintf('\n  ─── M5 FINAL: miss=%.1f cm, t=%.1fs, iters=%d, coll=%d ───\n\n', ...
    results.miss_distance, results.t_total, results.iterations, results.n_collisions);
end

% ─────────────────────────────────────────────────────────────────────────
function [n_arm, n_flt] = count_collisions(x_bar, t_f_bar, obstacles, p, scp)
    N = p.N; n_arm = 0;
    for k = 1:scp.M
        q_k = x_bar(1:N, k);
        [jpos_k, ~, ee_k] = forward_kinematics(q_k, p);
        for lnk = 1:N
            qr.type = 'link'; qr.p1 = jpos_k(:,lnk); qr.p2 = jpos_k(:,lnk+1);
            [c,~] = check_collision(qr, obstacles, p);
            if c, n_arm = n_arm + 1; end
        end
        alpha = sum(q_k);
        R = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];
        op.type = 'object'; op.pos = ee_k + R*p.obj.r_gc; op.theta = alpha;
        [c,~] = check_collision(op, obstacles, p);
        if c, n_arm = n_arm + 1; end
    end
    n_flt = 0;
    q_rel = x_bar(1:N,end); qd_rel = x_bar(N+1:end,end);
    rc = release_condition(q_rel, qd_rel, p);
    if ~isnan(rc.t_land)
        for ti = linspace(0, rc.t_land, 50)
            bq.type = 'object';
            bq.pos  = rc.obj_pos + [rc.obj_vel(1); rc.obj_vel(2) - 0.5*p.g*ti]*ti;
            bq.theta = rc.obj_theta + rc.obj_omega*ti;
            [c,~] = check_collision(bq, obstacles, p);
            if c, n_flt = n_flt + 1; end
        end
    end
end

function r = make_empty_result(method, config, errmsg)
    r.method = method; r.config = config; r.error = errmsg;
    r.miss_distance = NaN; r.t_total = NaN; r.iterations = 0;
    r.energy = NaN; r.t_release = NaN; r.n_collisions = NaN;
    r.x_land = NaN; r.collision_free = false;
end