% main_week2.m
% ===========================================================================
% WEEK 2: Single-Shooting Trajectory Optimization with fmincon
% ===========================================================================
% Master script for Week 2 of robotic arm throwing project.
% Optimizes torque trajectories τ(t) to minimize landing miss distance
% while satisfying joint limits, torque limits, and obstacle avoidance.
%
% PARAMETERIZATION: Piecewise-constant torques (M intervals)
% METHOD: MATLAB's fmincon with interior-point algorithm
% CONFIGURATIONS: Simple, Moderate, Hard (from Week 1)
%
% OUTPUT:
%   - Comparison table: Week 1 vs Week 2 results
%   - Convergence plots for each configuration
%   - Optimized torque profiles
%   - Animated trajectories
% ===========================================================================

clear; close all; clc;

%% ═══════════════════════════════════════════════════════════════════════
%%  INITIALIZATION
%% ═══════════════════════════════════════════════════════════════════════

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 2: SINGLE-SHOOTING TRAJECTORY OPTIMIZATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Add paths ─────────────────────────────────────────────────────────────
% Add Week 1 functions (from project directory)
% Add Week 2 functions (current directory)

addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week2\');
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');

%% ── Load parameters ───────────────────────────────────────────────────────
p = params();

% Week 2 optimization parameters
p.opt.M = 10;  % Number of piecewise-constant torque intervals
p.opt.collision_margin = 0.0;  % Safety margin [m] (0 = just avoid collision)

% Multi-objective weights
p.opt.w_position = 1000;  % High weight on landing accuracy
p.opt.w_energy = 1;       % Low weight on energy (regularization)

% Release time bounds
p.opt.t_release_min = 0.8;   % Minimum release time [s]
p.opt.t_release_max = 1.5;   % Maximum release time [s]

fprintf('[1/5] Parameters loaded:\n');
fprintf('  - Target distance d = %.3f m\n', p.task.d);
fprintf('  - Piecewise intervals M = %d\n', p.opt.M);
fprintf('  - Decision variables: z ∈ R^%d (%d torques + 1 release time)\n', ...
    4*p.opt.M + 1, 4*p.opt.M);
fprintf('  - Objective weights: position=%.0f, energy=%.1f\n', ...
    p.opt.w_position, p.opt.w_energy);
fprintf('  - Release time bounds: [%.1f, %.1f] s\n', ...
    p.opt.t_release_min, p.opt.t_release_max);
fprintf('  - Collision margin = %.3f m\n', p.opt.collision_margin);
fprintf('\n');

%% ── Load or generate dynamics ─────────────────────────────────────────────
fprintf('[2/5] Checking dynamics files...\n');

% Check if M_func.m exists (Week 1 generated files)
if ~exist('M_func.m', 'file')
    fprintf('  ⚠ Dynamics files not found. Running derive_dynamics.m...\n');
    derive_dynamics(p);
else
    fprintf('  ✓ Dynamics files found (M_func.m, C_func.m, G_func.m)\n');
end
fprintf('\n');

%% ── Define configurations to optimize ─────────────────────────────────────
configs = {'simple', 'moderate', 'hard'};
n_configs = length(configs);

% Storage for results
results = struct();

% Week 1 baseline results (for comparison)
week1_baseline.simple = struct('x_land', 179.9, 'miss_cm', 117.79);
week1_baseline.moderate = struct('x_land', 1000, 'miss_cm', 1000);
week1_baseline.hard = struct('x_land', 1000, 'miss_cm', 1000);

%% ═══════════════════════════════════════════════════════════════════════
%%  OPTIMIZATION LOOP (3 CONFIGURATIONS)
%% ═══════════════════════════════════════════════════════════════════════

for c_idx = 1:n_configs
    config_name = configs{c_idx};
    
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('  CONFIGURATION %d/%d: %s\n', c_idx, n_configs, upper(config_name));
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('\n');
    
    %% ── Load obstacle configuration ───────────────────────────────────────
    fprintf('[3/%d] Loading obstacles...\n', n_configs+2);
    obstacles = load_obstacle_config(config_name, p);
    p.obstacles = obstacles;
    fprintf('  Obstacles loaded: %d objects\n', length(obstacles));
    fprintf('\n');
    
    %% ── Create initial guess ──────────────────────────────────────────────
    fprintf('[4/%d] Creating initial guess...\n', n_configs+2);
    z0 = initialize_guess(p);
    
    % Test objective and constraints at initial guess
    fprintf('  Testing initial guess:\n');
    [J0, arm_coll0, flight_coll0, info0] = shooting_objective(z0, p);
    fprintf('    Initial objective J0 = %.6e\n', J0);
    fprintf('    Initial landing x = %.3f m, miss = %.2f cm\n', ...
        info0.x_land, abs(info0.x_land - p.task.d)*100);
    fprintf('    Initial energy = %.2f J\n', info0.E_total);
    fprintf('    Initial release time = %.2f s\n', info0.t_release);
    
    [c0, ceq0] = shooting_constraints(z0, p);
    fprintf('    Initial constraint violations: %d (max = %.4f)\n', ...
        sum(c0 > 0), max([0; c0]));
    fprintf('\n');
    
    %% ── Setup fmincon optimization ────────────────────────────────────────
    fprintf('[5/%d] Setting up fmincon...\n', n_configs+2);
    
    % Bounds: torque limits + release time bounds
    lb = [repmat(p.lim.tau_min, p.opt.M, 1); p.opt.t_release_min];  % [4M+1×1]
    ub = [repmat(p.lim.tau_max, p.opt.M, 1); p.opt.t_release_max];  % [4M+1×1]
    
    % fmincon options
    options = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'interior-point', ...
        'MaxIterations', 200, ...
        'MaxFunctionEvaluations', 10000, ...
        'ConstraintTolerance', 1e-4, ...
        'OptimalityTolerance', 1e-4, ...
        'StepTolerance', 1e-8, ...
        'FiniteDifferenceType', 'forward', ...
        'FiniteDifferenceStepSize', 1e-6, ...
        'UseParallel', false, ...
        'PlotFcn', @optimplotfval);
    
    fprintf('  Algorithm: %s\n', options.Algorithm);
    fprintf('  Max iterations: %d\n', options.MaxIterations);
    fprintf('  Max function evals: %d\n', options.MaxFunctionEvaluations);
    fprintf('\n');
    
    %% ── Run optimization ──────────────────────────────────────────────────
    fprintf('  Starting optimization...\n');
    fprintf('  (This may take several minutes)\n');
    fprintf('\n');
    
    tic;
    [z_opt, J_opt, exitflag, output] = fmincon(...
        @(z) shooting_objective(z, p), ...  % objective
        z0, ...                              % initial guess
        [], [], ...                          % A, b (no linear ineq)
        [], [], ...                          % Aeq, beq (no linear eq)
        lb, ub, ...                          % bounds
        @(z) shooting_constraints(z, p), ... % nonlinear constraints
        options);
    opt_time = toc;
    
    %% ── Process results ───────────────────────────────────────────────────
    fprintf('\n');
    fprintf('  Optimization complete!\n');
    fprintf('  Exit flag: %d\n', exitflag);
    fprintf('  Iterations: %d\n', output.iterations);
    fprintf('  Function evaluations: %d\n', output.funcCount);
    fprintf('  Time elapsed: %.2f s\n', opt_time);
    fprintf('\n');
    
    % Get final info
    [J_final, ~, ~, info_final] = shooting_objective(z_opt, p);
    
    % Store results
    results.(config_name).z_opt = z_opt;
    results.(config_name).J_opt = J_opt;
    results.(config_name).exitflag = exitflag;
    results.(config_name).iterations = output.iterations;
    results.(config_name).funcCount = output.funcCount;
    results.(config_name).time_s = opt_time;
    results.(config_name).x_land = info_final.x_land;
    results.(config_name).miss_cm = abs(info_final.x_land - p.task.d) * 100;
    results.(config_name).E_total = info_final.E_total;
    results.(config_name).t_release = info_final.t_release;
    
    % Check final constraint satisfaction
    [c_final, ~] = shooting_constraints(z_opt, p);
    results.(config_name).max_violation = max([0; c_final]);
    
    fprintf('  ═══ OPTIMIZED RESULTS ═══\n');
    fprintf('  Landing x = %.3f m\n', info_final.x_land);
    fprintf('  Miss distance = %.2f cm\n', results.(config_name).miss_cm);
    fprintf('  Energy consumed = %.2f J\n', info_final.E_total);
    fprintf('  Optimized release time = %.2f s\n', info_final.t_release);
    fprintf('  Max constraint violation = %.4e\n', results.(config_name).max_violation);
    fprintf('\n');
    
    %% ── Animate optimized trajectory ──────────────────────────────────────
    fprintf('  Animating optimized trajectory...\n');
    %animate_optimized_trajectory(z_opt, p, config_name);
    
    %% ── Save results ──────────────────────────────────────────────────────
    save_path = sprintf('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week2\week2_results_%s.mat', config_name);
    save(save_path, 'z_opt', 'J_opt', 'exitflag', 'output', 'info_final', 'p');
    fprintf('  Results saved: %s\n', save_path);
    fprintf('\n');

    disp('Script paused. Press any key to continue...');
    pause; 
    disp('Resuming...');

end

%% ═══════════════════════════════════════════════════════════════════════
%%  COMPARISON TABLE & SUMMARY
%% ═══════════════════════════════════════════════════════════════════════

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 1 vs WEEK 2 COMPARISON\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('%-12s | %12s | %12s | %12s | %8s | %8s\n', ...
    'Config', 'Week1 Miss', 'Week2 Miss', 'Improvement', 'Iters', 'Time(s)');
fprintf('%s\n', repmat('─', 1, 85));

for c_idx = 1:n_configs
    config_name = configs{c_idx};
    
    week1_miss = week1_baseline.(config_name).miss_cm;
    week2_miss = results.(config_name).miss_cm;
    improvement = week1_miss - week2_miss;
    
    fprintf('%-12s | %10.2f cm | %10.2f cm | %10.2f cm | %8d | %8.1f\n', ...
        upper(config_name), week1_miss, week2_miss, improvement, ...
        results.(config_name).iterations, results.(config_name).time_s);
end

fprintf('%s\n', repmat('─', 1, 85));
fprintf('\n');

%% ── Save all results ──────────────────────────────────────────────────────
save('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week2\week2_all_results.mat', 'results', 'week1_baseline', 'p');
fprintf('All results saved: week2_all_results.mat\n');

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 2 COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');
