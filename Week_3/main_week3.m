% main_week3.m
% ===========================================================================
% WEEK 3: Direct Collocation Trajectory Optimization with CasADi + IPOPT
% ===========================================================================
% Master script for Week 3 of robotic arm throwing project.
% Uses direct collocation to discretize both state and control, converting
% the continuous-time optimal control problem into a large sparse NLP.
%
% METHOD: CasADi symbolic framework + IPOPT interior-point solver
% ADVANTAGES: Exact gradients, sparse structure, better convergence
% CONFIGURATIONS: Simple, Moderate, Hard (from Weeks 1-2)
%
% OUTPUT:
%   - Comparison table: Week 1 vs Week 2 vs Week 3 results
%   - Convergence plots for each configuration
%   - Optimized state and torque trajectories
%   - Animated trajectories
% ===========================================================================

clear; close all; clc;

%% ═══════════════════════════════════════════════════════════════════════
%%  INITIALIZATION
%% ═══════════════════════════════════════════════════════════════════════

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3: DIRECT COLLOCATION TRAJECTORY OPTIMIZATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Check CasADi availability ─────────────────────────────────────────────
fprintf('[1/6] Checking CasADi installation...\n');

try
    import casadi.*
    fprintf('  ✓ CasADi loaded successfully\n');
    fprintf('  Version: %s\n', CasadiMeta.version());
catch ME
    fprintf('  ✗ CasADi not found!\n');
    fprintf('  Error: %s\n', ME.message);
    fprintf('\n');
    fprintf('  Please install CasADi from: https://web.casadi.org\n');
    fprintf('  Then add to MATLAB path: addpath(''path/to/casadi'')\n');
    return;
end
fprintf('\n');

%% ── Add paths ─────────────────────────────────────────────────────────────
% Add Week 1 functions (from project directory)
% Add Week 2 functions (for comparison)
% Add Week 3 functions (current directory)

addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');                     % Week 1 functions
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week2\');          % Week 2 functions
% Current directory already in path

%% ── Load parameters ───────────────────────────────────────────────────────
fprintf('[2/6] Loading parameters...\n');

p = params();

% Week 3 optimization parameters
p.opt.M = 60;  % Number of collocation points
p.opt.collision_margin = 0.0;  % Safety margin [m]

% Multi-objective weights
% KEY INSIGHT: Collision avoidance MUST dominate all other objectives.
% The optimizer should NEVER accept a collision to improve landing accuracy.
p.opt.w_position = 1000;    % Reduced from 1000 (still important but not dominant)
p.opt.w_energy = 0.1;      % Reduced (just regularization, not a real objective)
p.opt.w_direction = 1.0;     % Reduced (soft preference, not critical)

% Release time bounds
p.opt.t_release_min = 0.8;   % Minimum release time [s]
p.opt.t_release_max = 2.0;   % Maximum release time [s]

% NLP solver options
p.opt.ipopt_max_iter = 5000;    % More iterations for complex problem
p.opt.ipopt_tol = 1e-5;
p.opt.ipopt_print_level = 5;

fprintf('  - Target distance d = %.3f m\n', p.task.d);
fprintf('  - Collocation points M = %d\n', p.opt.M);
fprintf('  - Decision variables: z ∈ R^%d (state + control + t_release)\n', ...
    8*p.opt.M + 4*p.opt.M + 1);
fprintf('  - State variables: q, qdot at %d points = %d vars\n', ...
    p.opt.M, 8*p.opt.M);
fprintf('  - Control variables: τ at %d points = %d vars\n', ...
    p.opt.M, 4*p.opt.M);
fprintf('  - Total: %d decision variables\n', 8*p.opt.M + 4*p.opt.M + 1);
fprintf('  - Objective weights: position=%.0f, energy=%.1f, direction=%.0f\n', ...
    p.opt.w_position, p.opt.w_energy, p.opt.w_direction);
fprintf('  - Release time bounds: [%.1f, %.1f] s\n', ...
    p.opt.t_release_min, p.opt.t_release_max);
fprintf('\n');

%% ── Load or generate dynamics ─────────────────────────────────────────────
fprintf('[3/6] Checking dynamics files...\n');

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

% Week 1 & 2 baseline results (for comparison)
week1_baseline.simple = struct('miss_cm', 117.79);
week1_baseline.moderate = struct('miss_cm', 1000.00);  % Infeasible
week1_baseline.hard = struct('miss_cm', 1000.00);      % Infeasible

week2_baseline.simple = struct('miss_cm', 78.87);
week2_baseline.moderate = struct('miss_cm', 50.88);
week2_baseline.hard = struct('miss_cm', 337.79);

fprintf('[4/6] Configurations to optimize: %d\n', n_configs);
for i = 1:n_configs
    fprintf('  %d. %s\n', i, upper(configs{i}));
end
fprintf('\n');

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
    fprintf('[5/%d] Loading obstacles...\n', n_configs+4);
    obstacles = load_obstacle_config(config_name, p);
    p.obstacles = obstacles;
    fprintf('  Obstacles loaded: %d objects\n', length(obstacles));
    fprintf('\n');
    
    %% ── Build NLP with CasADi ─────────────────────────────────────────────
    fprintf('[6/%d] Building direct collocation NLP...\n', n_configs+4);
    fprintf('  (This creates symbolic expressions for dynamics and constraints)\n');
    
    tic;
    nlp = build_collocation_nlp(p);
    build_time = toc;
    
    fprintf('  ✓ NLP built in %.2f s\n', build_time);
    fprintf('  Decision variables: %d\n', nlp.n_vars);
    fprintf('  Inequality constraints: %d\n', nlp.n_ineq);
    fprintf('  Equality constraints: %d\n', nlp.n_eq);
    fprintf('\n');
    
    %% ── Create initial guess ──────────────────────────────────────────────
    fprintf('  Creating initial guess...\n');
    z0 = initialize_collocation_guess(nlp, p);
    fprintf('  ✓ Initial guess created: z0 ∈ R^%d\n', length(z0));
    fprintf('\n');
    
    %% ── Solve NLP with IPOPT ──────────────────────────────────────────────
    fprintf('  Starting IPOPT optimization...\n');
    fprintf('  (This may take several minutes)\n');
    fprintf('\n');
    
    tic;
    sol = solve_collocation_nlp(nlp, z0, p);
    opt_time = toc;
    
    %% ── Process results ───────────────────────────────────────────────────
    fprintf('\n');
    fprintf('  Optimization complete!\n');
    fprintf('  IPOPT status: %s\n', sol.stats.return_status);
    fprintf('  Iterations: %d\n', sol.stats.iter_count);
    fprintf('  Time elapsed: %.2f s\n', opt_time);
    fprintf('\n');
    
    % Extract solution (get each variable separately and concatenate)
    q_opt = full(sol.value(nlp.q));
    qdot_opt = full(sol.value(nlp.qdot));
    tau_opt = full(sol.value(nlp.tau));
    t_release_opt = full(sol.value(nlp.t_release));
    
    % Concatenate into single vector z_opt
    z_opt = [q_opt(:); qdot_opt(:); tau_opt(:); t_release_opt];
    
    % Unpack solution into trajectories
    traj_opt = unpack_collocation_solution(z_opt, nlp, p);
    
    % Store results
    results.(config_name).z_opt = z_opt;
    results.(config_name).traj = traj_opt;
    results.(config_name).ipopt_status = sol.stats.return_status;
    results.(config_name).iterations = sol.stats.iter_count;
    results.(config_name).time_s = opt_time;
    results.(config_name).x_land = traj_opt.x_land;
    results.(config_name).miss_cm = abs(traj_opt.x_land - p.task.d) * 100;
    results.(config_name).E_total = traj_opt.E_total;
    results.(config_name).t_release = traj_opt.t_release;
    
    fprintf('  ═══ OPTIMIZED RESULTS ═══\n');
    fprintf('  Landing x = %.3f m\n', traj_opt.x_land);
    fprintf('  Miss distance = %.2f cm\n', results.(config_name).miss_cm);
    fprintf('  Energy consumed = %.2f J\n', traj_opt.E_total);
    fprintf('  Optimized release time = %.2f s\n', traj_opt.t_release);
    fprintf('\n');
    
    %% ── Save results ──────────────────────────────────────────────────────
    save_path = sprintf('week3_results_%s.mat', config_name);
    save(save_path, 'z_opt', 'traj_opt', 'sol', 'nlp', 'p');
    fprintf('  Results saved: %s\n', save_path);
    fprintf('\n');
    
    % Pause for user review before next config
    if c_idx < n_configs
        fprintf('  Press any key to continue to next configuration...\n');
        %pause;
    end
end

%% ═══════════════════════════════════════════════════════════════════════
%%  COMPARISON TABLE & SUMMARY
%% ═══════════════════════════════════════════════════════════════════════

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 1 vs WEEK 2 vs WEEK 3 COMPARISON\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('%-12s | %12s | %12s | %12s | %12s | %8s\n', ...
    'Config', 'Week1 Miss', 'Week2 Miss', 'Week3 Miss', 'W2→W3 Δ', 'Time(s)');
fprintf('%s\n', repmat('─', 1, 95));

for c_idx = 1:n_configs
    config_name = configs{c_idx};
    
    week1_miss = week1_baseline.(config_name).miss_cm;
    week2_miss = week2_baseline.(config_name).miss_cm;
    week3_miss = results.(config_name).miss_cm;
    improvement = week2_miss - week3_miss;
    
    fprintf('%-12s | %10.2f cm | %10.2f cm | %10.2f cm | %10.2f cm | %8.1f\n', ...
        upper(config_name), week1_miss, week2_miss, week3_miss, improvement, ...
        results.(config_name).time_s);
end

fprintf('%s\n', repmat('─', 1, 95));
fprintf('\n');

%% ── Save all results ──────────────────────────────────────────────────────
save('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\week3_all_results.mat', 'results', ...
    'week1_baseline', 'week2_baseline', 'p');
fprintf('All results saved: week3_all_results.mat\n');

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3 COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');