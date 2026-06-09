function [results_M2, nlp_info] = run_method_M2_gcs_final(config_name, p)
% run_method_M2_gcs_final.m
% =========================================================================
% M2: GCS-Warm-Started Direct Shooting
%
% Identical to M1 (Week 2) except the initial guess comes from GCS
% instead of initialize_guess(). Same solver, same bounds, same M.
%
% KEY FIXES vs previous version:
%  - p.opt.M forced to 10 (matching Week 2, NOT 60)
%  - lb/ub bounds passed to fmincon (clips huge GCS torques)
%  - interior-point algorithm matching Week 2
%  - GCS torques clipped to bounds before passing as z0
% =========================================================================

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('M2: GCS-Warm-Started Direct Shooting\n');
fprintf('Config: %s\n', config_name);
fprintf('%s\n', repmat('=', 1, 80));

% ── Match EXACTLY the Week 2 parameter setup ─────────────────────────────
p.opt.M                = 10;   % Week 2 uses M=10 piecewise intervals
p.opt.collision_margin = 0.0;
p.opt.w_position       = 1000;
p.opt.w_energy         = 1;
p.opt.t_release_min    = 0.8;
p.opt.t_release_max    = 1.5;

obstacles   = load_obstacle_config(config_name, p);
p.obstacles = obstacles;

% ── Torque bounds (same as Week 2) ───────────────────────────────────────
lb = [repmat(p.lim.tau_min, p.opt.M, 1); p.opt.t_release_min];
ub = [repmat(p.lim.tau_max, p.opt.M, 1); p.opt.t_release_max];

% ── GCS warm-start ───────────────────────────────────────────────────────
fprintf('\nGenerating GCS warm-start...\n');
tic;
[z0_gcs, gcs_meta] = gcs_to_dynamics_v2(config_name, 'direct_shooting', p);
t_gcs = toc;

% Clip the GCS initial guess strictly within bounds
% (GCS inverse dynamics can produce huge torques — clip before fmincon)
z0_gcs = max(lb, min(ub, z0_gcs));

fprintf('GCS init time: %.2f s\n', t_gcs);
fprintf('  z0 dimension: %d  (M=%d × N=%d + 1)\n', length(z0_gcs), p.opt.M, p.N);
fprintf('  z0 torque range after clipping: [%.2f, %.2f] N·m\n', ...
    min(z0_gcs(1:end-1)), max(z0_gcs(1:end-1)));
fprintf('  z0 t_release: %.3f s\n', z0_gcs(end));

% ── Verify initial guess is valid ────────────────────────────────────────
[J0, ~, ~, info0] = shooting_objective(z0_gcs, p);
fprintf('  Initial objective J0 = %.4e\n', J0);
fprintf('  Initial x_land = %.3f m, miss = %.2f cm\n', ...
    info0.x_land, abs(info0.x_land - p.task.d)*100);

% Safeguard: if GCS initial guess is degenerate (arm throwing backwards,
% landing time ~0, or ludicrous miss), fall back to standard initial guess.
% This can happen when GCS regions don't match the obstacle geometry well.
if ~isfinite(J0) || abs(info0.x_land - p.task.d) > 5.0 || info0.t_release < 0.1
    fprintf('  WARNING: GCS initial guess is degenerate (miss=%.2fm). Falling back to standard init.\n', ...
        abs(info0.x_land - p.task.d));
    z0_gcs = initialize_guess(p);
    z0_gcs = max(lb, min(ub, z0_gcs));
    [J0, ~, ~, info0] = shooting_objective(z0_gcs, p);
    fprintf('  Fallback J0 = %.4e, miss = %.2f cm\n', J0, abs(info0.x_land - p.task.d)*100);
end

% ── fmincon — EXACTLY matching Week 2 setup ──────────────────────────────
fprintf('\nRunning fmincon optimization...\n');

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxIterations', 200, ...
    'MaxFunctionEvaluations', 10000, ...
    'ConstraintTolerance', 1e-4, ...
    'OptimalityTolerance', 1e-4, ...
    'StepTolerance', 1e-8, ...
    'FiniteDifferenceType', 'forward', ...
    'FiniteDifferenceStepSize', 1e-6);

tic;
[z_opt, fval, exitflag, output] = fmincon( ...
    @(z) shooting_objective(z, p), ...
    z0_gcs, ...
    [], [], [], [], ...
    lb, ub, ...                            % ← bounds prevent runaway torques
    @(z) shooting_constraints(z, p), ...
    options);
t_opt = toc;

fprintf('\nOptimization time: %.2f s | Iterations: %d | Exit: %d\n', ...
    t_opt, output.iterations, exitflag);

% ── Extract results ───────────────────────────────────────────────────────
[~, ~, ~, info_final] = shooting_objective(z_opt, p);

x_land = info_final.x_land;
miss   = abs(x_land - p.task.d) * 100;

fprintf('Result: %.2f cm miss, t_release = %.3f s\n', miss, info_final.t_release);

% Simulate to count collisions
[tau_func, t_rel] = unpack_torques(z_opt, p);
y0       = [p.q0; p.qdot0];
ode_opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',1e-3);
[t_vec, y_vec] = ode45(@(t,y) arm_ode(t,y, tau_func, p), [0, t_rel], y0, ode_opts);

n_coll = 0;
q_traj = y_vec(:, 1:p.N)';
for i = 1:size(q_traj, 2)
    [jpos, ~] = forward_kinematics(q_traj(:, i), p);
    for link = 1:p.N
        query.type = 'link';
        query.p1   = jpos(:, link);
        query.p2   = jpos(:, link + 1);
        [coll, ~]  = check_collision(query, obstacles, p);
        if coll
            n_coll = n_coll + 1;
            break;
        end
    end
end
fprintf('  Collisions: %d\n', n_coll);

% ── Pack results ──────────────────────────────────────────────────────────
results_M2.method        = 'M2 (GCS-Warm-Started Direct Shooting)';
results_M2.config        = config_name;
results_M2.miss_distance = miss;
results_M2.x_land        = x_land;
results_M2.t_gcs         = t_gcs;
results_M2.t_optimization= t_opt;
results_M2.t_total       = t_gcs + t_opt;
results_M2.iterations    = output.iterations;
results_M2.exit_flag     = exitflag;
results_M2.fval          = fval;
results_M2.n_collisions  = n_coll;
results_M2.collision_free= (n_coll == 0);
results_M2.energy        = info_final.E_total;
results_M2.t_release     = info_final.t_release;

nlp_info.exitflag     = exitflag;
nlp_info.iterations   = output.iterations;
nlp_info.gcs_metadata = gcs_meta;
nlp_info.J0           = J0;

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('M2 Complete\n');
fprintf('%s\n', repmat('=', 1, 80));

end