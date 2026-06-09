% main_week5.m
% =========================================================================
% WEEK 5: Sequential Convex Programming (M5 & M6)
% =========================================================================
% Master script for Week 5 of the robotic arm throwing project.
%
% Implements and runs:
%   M5 — Standard SCP (backward-swing initialisation)
%   M6 — GCS-warm-started SCP (GCS workspace planner initialisation)
%
% Then runs the 6-method comparison (M1–M6) across all three obstacle
% configurations.
%
% PREREQUISITES:
%   - Week 1 functions on MATLAB path (params, dynamics, kinematics, ...)
%   - Week 3 functions on path (build_collocation_nlp, initialize_collocation_guess, ...)
%   - Week 4 functions on path (gcs_solver_v2, gcs_to_dynamics_v2)
%   - Week 5 functions in current directory
%   - .mat result files for M1–M4 (comparison_simple.mat, etc.)
%
% OUTPUT:
%   - Comparison tables for all configurations
%   - Saved results: week5_results.mat
% =========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 5: SEQUENTIAL CONVEX PROGRAMMING (M5 & M6)\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  Date: %s\n', datestr(now));
fprintf('═══════════════════════════════════════════════════════════════\n\n');

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

% Week 5 functions (current directory — SCP)
% Already on path if running from Week_5 directory.

% Result .mat files — update if stored elsewhere
mat_dir = 'C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\';

%% ── LOAD PARAMETERS ─────────────────────────────────────────────────────
fprintf('[1/5] Loading parameters...\n');
p = params();

% Collocation parameters (needed for initialisation helper)
p.opt.M = 60;
p.opt.collision_margin = 0.0;
p.opt.w_position  = 1000;
p.opt.w_energy    = 0.1;
p.opt.w_direction = 1.0;
p.opt.t_release_min = 0.8;
p.opt.t_release_max = 2.0;
p.opt.ipopt_max_iter = 5000;
p.opt.ipopt_tol = 1e-5;
p.opt.ipopt_print_level = 0;
fprintf('  ✓ Parameters loaded\n\n');

%% ── CONFIGURATIONS TO RUN ───────────────────────────────────────────────
configs = {'simple', 'moderate', 'hard'};

% Storage
all_results = struct();

%% ── RUN M5 & M6 ON EACH CONFIGURATION ──────────────────────────────────
for ci = 1:length(configs)
    cfg = configs{ci};
    fprintf('\n');
    fprintf('████████████████████████████████████████████████████████████████\n');
    fprintf('  CONFIGURATION: %s  (%d/%d)\n', upper(cfg), ci, length(configs));
    fprintf('████████████████████████████████████████████████████████████████\n');

    % ── Run M5 (Standard SCP) ────────────────────────────────────────
    fprintf('\n[2/5] Running M5 (Standard SCP) on %s...\n', cfg);
    try
        [res_M5, log_M5] = run_method_M5_scp(cfg, p);
        all_results.(cfg).M5 = res_M5;
        all_results.(cfg).M5_log = log_M5;
    catch ME
        fprintf('  ✗ M5 FAILED on %s: %s\n', cfg, ME.message);
        all_results.(cfg).M5 = struct('miss_distance',NaN,'t_total',NaN,...
            'iterations',0,'energy',NaN,'n_collisions',NaN,...
            't_release',NaN,'x_land',NaN,'error',ME.message);
        res_M5 = all_results.(cfg).M5;
    end

    % ── Run M6 (GCS-warm-started SCP) ────────────────────────────────
    fprintf('\n[3/5] Running M6 (GCS SCP) on %s...\n', cfg);
    try
        [res_M6, log_M6] = run_method_M6_gcs_scp(cfg, p);
        all_results.(cfg).M6 = res_M6;
        all_results.(cfg).M6_log = log_M6;
    catch ME
        fprintf('  ✗ M6 FAILED on %s: %s\n', cfg, ME.message);
        all_results.(cfg).M6 = struct('miss_distance',NaN,'t_total',NaN,...
            'iterations',0,'energy',NaN,'n_collisions',NaN,...
            't_release',NaN,'x_land',NaN,'error',ME.message);
        res_M6 = all_results.(cfg).M6;
    end

    % ── 6-Method comparison ──────────────────────────────────────────
    fprintf('\n[4/5] Running 6-method comparison for %s...\n', cfg);
    comp_file = fullfile(mat_dir, sprintf('comparison_%s.mat', cfg));
    w2_file   = fullfile(mat_dir, 'Week_2\week2_all_results.mat');
    w3_file   = fullfile(mat_dir, 'Week_3\week3_all_results.mat');

    try
        compare_methods_all(cfg, res_M5, res_M6, comp_file, w2_file, w3_file);
    catch ME
        fprintf('  ⚠ Comparison failed for %s: %s\n', cfg, ME.message);
        fprintf('    (M5/M6 results are still saved independently)\n');
    end
end

%% ── SAVE ALL RESULTS ────────────────────────────────────────────────────
fprintf('\n[5/5] Saving results...\n');
save('week5_results.mat', 'all_results', 'p');
fprintf('  ✓ Saved to week5_results.mat\n');

%% ── FINAL SUMMARY ──────────────────────────────────────────────────────
fprintf('\n');
fprintf('════════════════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 5 COMPLETE — SUMMARY\n');
fprintf('════════════════════════════════════════════════════════════════════════\n\n');

fprintf('  %-10s  %-6s  %8s  %8s  %6s  %6s\n', ...
    'Config', 'Method', 'Miss(cm)', 'Time(s)', 'Iters', 'Coll');
fprintf('  %s\n', repmat('─', 1, 56));

for ci = 1:length(configs)
    cfg = configs{ci};
    for m = {'M5', 'M6'}
        mid = m{1};
        if isfield(all_results.(cfg), mid)
            r = all_results.(cfg).(mid);
            fprintf('  %-10s  %-6s  %8.1f  %8.2f  %6d  %6d\n', ...
                cfg, mid, r.miss_distance, r.t_total, ...
                r.iterations, r.n_collisions);
        end
    end
end

fprintf('  %s\n', repmat('─', 1, 56));
fprintf('\n  All results saved to week5_results.mat\n');
fprintf('  Comparison tables saved to comparison_all_<config>.mat\n\n');