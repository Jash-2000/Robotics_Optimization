% test_week5_setup.m
% =========================================================================
% SMOKE TEST: Validate all Week 5 SCP components before running the full
% optimisation.  Run this first to catch path / signature / dimension bugs.
% =========================================================================

clear; close all; clc;

%% ── PATH SETUP ──────────────────────────────────────────────────────────
% IMPORTANT: Update these paths to match your directory structure.

% Week 1 core functions (params, dynamics, kinematics, collision, etc.)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');

% Week 2 functions (shooting)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_2\');

% Week 3 functions (collocation + CasADi)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\');

% Week 4 functions (GCS)
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_4\');


fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 5 SETUP VALIDATION  —  SCP Components\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

n_pass = 0;
n_fail = 0;

%% ── Test 1: Parameters ──────────────────────────────────────────────────
fprintf('[Test 1/8] Loading parameters...\n');
try
    p = params();
    fprintf('  ✓ params() loaded.  N=%d, target=%.3f m\n', p.N, p.task.d);
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1; return;
end

%% ── Test 2: Dynamics wrapper ────────────────────────────────────────────
fprintf('\n[Test 2/8] arm_dynamics_continuous...\n');
try
    x0 = [p.q0; p.qdot0];
    u0 = zeros(p.N, 1);
    xdot = arm_dynamics_continuous(x0, u0, p);
    assert(numel(xdot) == 8, 'xdot should be 8×1');
    fprintf('  ✓ f(x0, 0) = [%.3f, ..., %.3f]  (8×1)\n', xdot(1), xdot(end));
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 3: Dynamics Jacobians ──────────────────────────────────────────
fprintf('\n[Test 3/8] scp_dynamics_jacobians...\n');
try
    [A, B] = scp_dynamics_jacobians(x0, u0, p);
    assert(all(size(A) == [8,8]), 'A should be 8×8');
    assert(all(size(B) == [8,4]), 'B should be 8×4');

    % B should have structure [0; Minv] (since dynamics are affine in u)
    % Top 4 rows of B should be ~0
    assert(max(abs(B(1:4,:)), [], 'all') < 1e-8, 'Top half of B should be zero');
    fprintf('  ✓ A: [8×8], B: [8×4]  (top of B is zero ✓)\n');
    fprintf('    ||A|| = %.3f,  ||B(5:8,:)|| = %.3f\n', norm(A,'fro'), norm(B(5:8,:),'fro'));
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 4: Collision linearisation ─────────────────────────────────────
fprintf('\n[Test 4/8] scp_collision_linearize...\n');
try
    obstacles = load_obstacle_config('simple', p);
    [Ac, bc, info] = scp_collision_linearize(p.q0, obstacles, p, 0.05);
    fprintf('  ✓ %d halfplane constraints for SIMPLE config\n', info.n_constraints);
    assert(info.n_constraints > 0, 'Should have at least 1 constraint');
    assert(size(Ac,2) == p.N, 'A_coll columns should equal N');
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 5: QP building ────────────────────────────────────────────────
fprintf('\n[Test 5/8] scp_build_subproblem (small M=5)...\n');
try
    obstacles = load_obstacle_config('simple', p);
    p.obstacles = obstacles;

    % Tiny SCP struct for testing
    scp_test.M = 5;
    scp_test.trust_radius = 0.5;
    scp_test.trust_radius_u = 5.0;
    scp_test.t_release_min = 0.8;
    scp_test.t_release_max = 2.5;
    scp_test.w_position = 500;
    scp_test.w_energy = 0.1;
    scp_test.w_trust = 5.0;
    scp_test.w_slack = 1e4;
    scp_test.collision_margin = 0.05;

    M_test = scp_test.M;
    N = p.N;

    % Create trivial reference trajectory
    x_bar = repmat([p.q0; p.qdot0], 1, M_test);
    u_bar = zeros(N, M_test);
    t_f_bar = 1.3;

    qp = scp_build_subproblem(x_bar, u_bar, t_f_bar, p, obstacles, scp_test);
    fprintf('  ✓ QP built:  nz=%d (core=%d, slacks=%d)\n', ...
        qp.nz, qp.nz_core, qp.ns);
    fprintf('    Aeq: [%d×%d],  Aiq: [%d×%d]\n', ...
        size(qp.Aeq,1), size(qp.Aeq,2), size(qp.Aiq,1), size(qp.Aiq,2));
    fprintf('    H symmetric: %s\n', mat2str(issymmetric(full(qp.H))));
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 6: QP solving ─────────────────────────────────────────────────
fprintf('\n[Test 6/8] scp_solve_subproblem...\n');
try
    [z_sol, qp_info] = scp_solve_subproblem(qp);
    if qp_info.feasible
        fprintf('  ✓ QP solved: exitflag=%d, fval=%.2e\n', ...
            qp_info.exitflag, qp_info.fval);
    else
        fprintf('  ⚠ QP infeasible (exitflag=%d) — fallback=%s\n', ...
            qp_info.exitflag, mat2str(qp_info.fallback));
    end
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 7: Cost evaluation ────────────────────────────────────────────
fprintf('\n[Test 7/8] scp_evaluate_cost...\n');
try
    scp_test.w_collision = 1e6;
    [J, ci] = scp_evaluate_cost(x_bar, u_bar, t_f_bar, p, obstacles, scp_test);
    fprintf('  ✓ J=%.2f  (pos=%.2f, energy=%.2f, coll=%.2f)\n', ...
        J, ci.J_pos, ci.J_energy, ci.J_coll);
    fprintf('    Miss: %.1f cm,  collisions: %d\n', ci.miss_cm, ci.n_collisions);
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Test 8: Trajectory initialisation ───────────────────────────────────
fprintf('\n[Test 8/8] scp_initialize_trajectory (standard)...\n');
try
    scp_init.M = 10;  % small for speed
    scp_init.t_release_min = 0.8;
    scp_init.t_release_max = 2.5;
    [x0, u0, t0] = scp_initialize_trajectory(p, scp_init, 'standard');
    assert(size(x0,1)==8 && size(x0,2)==10, 'x0 should be 8×10');
    assert(size(u0,1)==4 && size(u0,2)==10, 'u0 should be 4×10');
    fprintf('  ✓ Trajectory: x [%d×%d], u [%d×%d], t_f=%.2f s\n', ...
        size(x0,1), size(x0,2), size(u0,1), size(u0,2), t0);
    n_pass = n_pass + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message); n_fail = n_fail + 1;
end

%% ── Summary ─────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
if n_fail == 0
    fprintf('  ALL %d TESTS PASSED ✓\n', n_pass);
else
    fprintf('  %d PASSED,  %d FAILED ✗\n', n_pass, n_fail);
end
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

if n_fail == 0
    fprintf('Week 5 implementation is ready.\n');
    fprintf('Run main_week5.m to execute M5 and M6 on all configurations.\n\n');
end