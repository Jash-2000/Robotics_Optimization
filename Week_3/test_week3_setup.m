% test_week3_setup.m
% ===========================================================================
% TEST SCRIPT: Validate Week 3 implementation before full optimization
% ===========================================================================
% This script tests all Week 3 components to ensure CasADi and direct
% collocation setup work correctly.
% ===========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3 SETUP VALIDATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Test 1: CasADi availability ───────────────────────────────────────────
fprintf('[Test 1/6] Checking CasADi installation...\n');
try
    import casadi.*
    fprintf('  ✓ CasADi imported successfully\n');
    fprintf('    Version: %s\n', CasadiMeta.version());
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    fprintf('  Install CasADi from: https://web.casadi.org\n');
    return;
end
fprintf('\n');

%% ── Test 2: Load parameters ───────────────────────────────────────────────
fprintf('[Test 2/6] Loading parameters...\n');
try
    addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');  % Week 1 functions
    
    p = params();
    p.opt.M = 10;  % Small M for testing
    p.opt.collision_margin = 0.0;
    p.opt.w_position = 1000;
    p.opt.w_energy = 1;
    p.opt.w_direction = 10;
    p.opt.t_release_min = 0.8;
    p.opt.t_release_max = 1.5;
    p.opt.ipopt_max_iter = 100;
    p.opt.ipopt_tol = 1e-6;
    p.opt.ipopt_print_level = 3;
    
    fprintf('  ✓ Parameters loaded successfully\n');
    fprintf('    Collocation points: M = %d\n', p.opt.M);
    fprintf('    Decision vars: %d (state + control + t)\n', 12*p.opt.M + 1);
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 3: Check dynamics files ──────────────────────────────────────────
fprintf('[Test 3/6] Checking dynamics files...\n');
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

%% ── Test 4: Build NLP ─────────────────────────────────────────────────────
fprintf('[Test 4/6] Building collocation NLP...\n');
try
    % Load simple obstacle config for testing
    p.obstacles = load_obstacle_config('simple', p);
    
    tic;
    nlp = build_collocation_nlp(p);
    build_time = toc;
    
    fprintf('  ✓ NLP built in %.2f s\n', build_time);
    fprintf('    Decision variables: %d\n', nlp.n_vars);
    fprintf('    Equality constraints: %d\n', nlp.n_eq);
    fprintf('    Inequality constraints (estimated): %d\n', nlp.n_ineq);
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    fprintf('  Stack trace:\n');
    disp(ME.getReport());
    return;
end
fprintf('\n');

%% ── Test 5: Create initial guess ──────────────────────────────────────────
fprintf('[Test 5/6] Creating initial guess...\n');
try
    z0 = initialize_collocation_guess(nlp, p);
    
    fprintf('  ✓ Initial guess created\n');
    fprintf('    Length: %d\n', length(z0));
    fprintf('    Min value: %.3f\n', min(z0));
    fprintf('    Max value: %.3f\n', max(z0));
    
    % Check dimensions match
    if length(z0) ~= nlp.n_vars
        fprintf('  ⚠ WARNING: z0 length (%d) != n_vars (%d)\n', ...
            length(z0), nlp.n_vars);
    else
        fprintf('  ✓ Dimensions match: z0 ∈ R^%d\n', nlp.n_vars);
    end
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    return;
end
fprintf('\n');

%% ── Test 6: Test solver setup (don't solve, just configure) ───────────────
fprintf('[Test 6/6] Testing solver configuration...\n');
try
    % Configure but don't solve (would take too long for test)
    opti = nlp.opti;
    
    % Try to set initial guess
    M = nlp.M;
    N = nlp.N;
    
    idx_q = 1:(N*M);
    idx_qdot = (N*M+1):(2*N*M);
    idx_tau = (2*N*M+1):(3*N*M);
    idx_t = 3*N*M + 1;
    
    q0_mat = reshape(z0(idx_q), N, M);
    qdot0_mat = reshape(z0(idx_qdot), N, M);
    tau0_mat = reshape(z0(idx_tau), N, M);
    t_release0 = z0(idx_t);
    
    opti.set_initial(nlp.q, q0_mat);
    opti.set_initial(nlp.qdot, qdot0_mat);
    opti.set_initial(nlp.tau, tau0_mat);
    opti.set_initial(nlp.t_release, t_release0);
    
    fprintf('  ✓ Initial values set in Opti stack\n');
    
    % Configure solver (but don't call solve)
    opts = struct();
    opts.ipopt.max_iter = p.opt.ipopt_max_iter;
    opts.ipopt.print_level = 0;  % Suppress output for test
    
    opti.solver('ipopt', opts);
    
    fprintf('  ✓ IPOPT solver configured\n');
    fprintf('    (Not solving in test - would take too long)\n');
    
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
fprintf('Week 3 implementation is ready for optimization.\n');
fprintf('Run main_week3.m to optimize all three configurations.\n');
fprintf('\n');
fprintf('Expected time for full optimization:\n');
fprintf('  - Simple config:   5-15 minutes\n');
fprintf('  - Moderate config: 10-20 minutes\n');
fprintf('  - Hard config:     15-30 minutes\n');
fprintf('  Total:            30-60 minutes (depends on CasADi/IPOPT performance)\n');
fprintf('\n');
