% debug_m5_single.m
% Run M5 on simple config with full error visibility — NO try/catch
clear; close all; clc;

addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_2\');
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\');
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_4\');

fprintf('=== STEP 1: params ===\n');
p = params();
p.opt.M = 60;
fprintf('  OK\n');

fprintf('=== STEP 2: load obstacles ===\n');
obstacles = load_obstacle_config('simple', p);
p.obstacles = obstacles;
p.current_config = 'hard';
fprintf('  OK\n');

fprintf('=== STEP 3: SCP params ===\n');
scp.M = 60;
scp.max_iters = 40;
scp.convergence_tol = 1e-3;
scp.trust_radius = 0.1;
scp.trust_radius_u = 1.0;
scp.trust_radius_min = 1e-3;
scp.trust_radius_max = 3.0;
scp.trust_radius_u_min = 0.5;
scp.trust_radius_u_max = 20.0;
scp.t_release_min = 0.8;
scp.t_release_max = 2.5;
scp.w_position = 500;
scp.w_energy = 0.1;
scp.w_trust = 5.0;
scp.w_slack = 1e4;
scp.w_collision = 1e6;
scp.collision_margin = 0.05;
fprintf('  OK\n');

fprintf('=== STEP 4: arm_dynamics_continuous (single eval) ===\n');
x0 = [p.q0; p.qdot0];
u0 = zeros(4,1);
xdot = arm_dynamics_continuous(x0, u0, p);
fprintf('  xdot = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', xdot);
fprintf('  OK\n');

fprintf('=== STEP 5: scp_initialize_trajectory ===\n');
[x_bar, u_bar, t_f_bar] = scp_initialize_trajectory(p, scp, 'standard');
fprintf('  x_bar NaN: %d, u_bar NaN: %d\n', nnz(isnan(x_bar)), nnz(isnan(u_bar)));
fprintf('  q range:    [%.3f, %.3f]\n', min(x_bar(1:4,:),[],'all'), max(x_bar(1:4,:),[],'all'));
fprintf('  qdot range: [%.3f, %.3f]\n', min(x_bar(5:8,:),[],'all'), max(x_bar(5:8,:),[],'all'));
fprintf('  t_f = %.2f\n', t_f_bar);
fprintf('  OK\n');

fprintf('=== STEP 6: scp_evaluate_cost ===\n');
[J, ci] = scp_evaluate_cost(x_bar, u_bar, t_f_bar, p, obstacles, scp);
fprintf('  J=%.2f, miss=%.1f cm, coll=%d\n', J, ci.miss_cm, ci.n_collisions);
fprintf('  OK\n');

fprintf('=== STEP 7: scp_build_subproblem ===\n');
qp = scp_build_subproblem(x_bar, u_bar, t_f_bar, p, obstacles, scp);
fprintf('  nz=%d, Aeq=[%dx%d], Aiq=[%dx%d]\n', ...
    qp.nz, size(qp.Aeq,1), size(qp.Aeq,2), size(qp.Aiq,1), size(qp.Aiq,2));
fprintf('  Bounds feasible: %s\n', mat2str(all(qp.lb <= qp.ub + 1e-8)));
fprintf('  OK\n');

fprintf('=== STEP 8: scp_solve_subproblem ===\n');
[z_sol, qp_info] = scp_solve_subproblem(qp);
fprintf('  exitflag=%d, empty=%s\n', qp_info.exitflag, mat2str(isempty(z_sol)));
fprintf('  OK\n');

fprintf('=== STEP 9: scp_unpack_solution ===\n');
[x_new, u_new] = scp_unpack_solution(z_sol, qp);
fprintf('  x_new NaN: %d\n', nnz(isnan(x_new)));
fprintf('  OK\n');

fprintf('=== STEP 10: scp_update_release_time ===\n');
t_f_new = scp_update_release_time(x_new, u_new, t_f_bar, p, obstacles, scp);
fprintf('  t_f_new = %.2f\n', t_f_new);
fprintf('  OK\n');

fprintf('\n=== ALL STEPS PASSED — SCP IS WORKING ===\n');
fprintf('Now run: [res, log] = run_method_M5_scp(''simple'', p);\n');