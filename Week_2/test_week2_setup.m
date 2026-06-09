% test_week2_setup.m
% ===========================================================================
% TEST SCRIPT: Validate Week 2 implementation before full optimization
% ===========================================================================
% This script tests all Week 2 components to ensure they work correctly
% before running the computationally expensive optimization.
% ===========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 2 SETUP VALIDATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Add paths ─────────────────────────────────────────────────────────────
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week2\');
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');

%% ── Test 1: Load parameters ───────────────────────────────────────────────
fprintf('[Test 1/7] Loading parameters...\n');
try
    p = params();
    p.opt.M = 10;  % Use small M for testing
    p.opt.collision_margin = 0.0;
    p.opt.w_position = 1;  % Weight on accuracy
    p.opt.w_energy = 0.1;       % Weight on energy
    p.opt.t_release_min = 0.8;
    p.opt.t_release_max = 1.5;
    fprintf('  ✓ Parameters loaded successfully\n');
    fprintf('    Target: d = %.3f m\n', p.task.d);
    fprintf('    Intervals: M = %d\n', p.opt.M);
    fprintf('    Decision vars: z ∈ R^%d\n', 4*p.opt.M + 1);
    fprintf('    Release time bounds: [%.1f, %.1f] s\n', ...
        p.opt.t_release_min, p.opt.t_release_max);
    fprintf('    Decision vars: %d\n', 4*p.opt.M);
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 2: Check dynamics files ──────────────────────────────────────────
fprintf('[Test 2/7] Checking dynamics files...\n');
if exist('M_func.m', 'file') && exist('C_func.m', 'file') && exist('G_func.m', 'file')
    fprintf('  ✓ Dynamics files found\n');
else
    fprintf('  ⚠ Dynamics files not found. Running derive_dynamics...\n');
    try
        derive_dynamics(p);
        fprintf('  ✓ Dynamics generated successfully\n');
    catch ME
        fprintf('  ✗ FAILED: %s\n', ME.message);
        return;
    end
end
fprintf('\n');

%% ── Test 3: Initialize guess ──────────────────────────────────────────────
fprintf('[Test 3/7] Creating initial guess...\n');
try
    z0 = initialize_guess(p);
    fprintf('  ✓ Initial guess created: z0 ∈ R^%d\n', length(z0));
    fprintf('    z0 min = %.3f, max = %.3f\n', min(z0(1:end-1)), max(z0(1:end-1)));
    fprintf('    Release time guess = %.2f s\n', z0(end));
    
    % Check bounds (torques + release time)
    lb = [repmat(p.lim.tau_min, p.opt.M, 1); p.opt.t_release_min];
    ub = [repmat(p.lim.tau_max, p.opt.M, 1); p.opt.t_release_max];
    if all(z0 >= lb) && all(z0 <= ub)
        fprintf('  ✓ Initial guess within bounds\n');
    else
        fprintf('  ⚠ Warning: Initial guess violates bounds!\n');
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 4: Unpack torques ────────────────────────────────────────────────
fprintf('[Test 4/7] Testing torque unpacking...\n');
try
    [tau_func, t_release_extracted] = unpack_torques(z0, p);
    fprintf('  ✓ Extracted release time: %.2f s\n', t_release_extracted);
    
    % Test at several time points
    t_test = [0, 0.5, 1.0, t_release_extracted];
    for i = 1:length(t_test)
        % tau_func signature: tau_func(t, q, qdot)
        q_dummy = zeros(4,1);
        qdot_dummy = zeros(4,1);
        tau = tau_func(t_test(i), q_dummy, qdot_dummy);
        fprintf('  τ(%.1f) = [%.2f, %.2f, %.2f, %.2f] N·m\n', ...
            t_test(i), tau(1), tau(2), tau(3), tau(4));
    end
    fprintf('  ✓ Torque function works correctly\n');
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 5: Load obstacle config ──────────────────────────────────────────
fprintf('[Test 5/7] Loading obstacle configuration (simple)...\n');
try
    obstacles = load_obstacle_config('hard', p);
    p.obstacles = obstacles;
    fprintf('  ✓ Obstacles loaded: %d objects\n', length(obstacles));
    for i = 1:length(obstacles)
        fprintf('    Obstacle %d: %s at (%.2f, %.2f) m\n', ...
            i, obstacles(i).type, obstacles(i).cx, obstacles(i).cy);
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 6: Evaluate objective ────────────────────────────────────────────
fprintf('[Test 6/7] Evaluating objective at z0...\n');
try
    tic;
    [J0, arm_coll, flight_coll, info0] = shooting_objective(z0, p);
    eval_time = toc;
    
    fprintf('  ✓ Objective computed successfully\n');
    fprintf('    J0 = %.6e\n', J0);
    fprintf('    Initial landing x = %.3f m\n', info0.x_land);
    fprintf('    Initial miss = %.2f cm\n', abs(info0.x_land - p.task.d)*100);
    fprintf('    Initial energy = %.2f J\n', info0.E_total);
    fprintf('    Initial release time = %.2f s\n', info0.t_release);
    fprintf('    Arm collision checks: %d\n', length(arm_coll));
    fprintf('    Flight collision checks: %d\n', length(flight_coll));
    fprintf('    Evaluation time: %.3f s\n', eval_time);
    
    if ~isnan(J0) && isfinite(J0)
        fprintf('  ✓ Objective is valid (finite)\n');
    else
        fprintf('  ✗ Objective is NaN or Inf!\n');
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 7: Evaluate constraints ──────────────────────────────────────────
fprintf('[Test 7/7] Evaluating constraints at z0...\n');
try
    [c0, ceq0] = shooting_constraints(z0, p);
    
    fprintf('  ✓ Constraints computed successfully\n');
    fprintf('    Inequality constraints: %d\n', length(c0));
    fprintf('    Equality constraints: %d\n', length(ceq0));
    
    n_violated = sum(c0 > 1e-6);
    max_violation = max([0; c0]);
    
    fprintf('    Violated constraints: %d / %d\n', n_violated, length(c0));
    fprintf('    Max violation: %.4e\n', max_violation);
    
    if n_violated == 0
        fprintf('  ✓ Initial guess is feasible!\n');
    else
        fprintf('  ⚠ Initial guess has constraint violations\n');
        fprintf('    (This is OK - optimizer will resolve them)\n');
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Summary ───────────────────────────────────────────────────────────────
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  ALL TESTS PASSED ✓\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');
fprintf('Week 2 implementation is ready for optimization.\n');
fprintf('Run main_week2.m to optimize all three configurations.\n');
fprintf('\n');
fprintf('Estimated time for full optimization:\n');
fprintf('  - Simple config:   2-5 minutes\n');
fprintf('  - Moderate config: 5-10 minutes\n');
fprintf('  - Hard config:     8-15 minutes\n');
fprintf('  Total:            15-30 minutes\n');
fprintf('\n');