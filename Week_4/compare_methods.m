function compare_methods(config_name, res_M2, res_M4, week3_mat_path, week2_mat_path)
% compare_methods.m
% =========================================================================
% Build a comparison table from ALREADY-COMPUTED results.
% Does NOT re-run any optimizer.
%
% USAGE:
%   % First run M2 and M4 individually:
%   p = params();
%   [res_M2, ~] = run_method_M2_gcs_final('simple', p);
%   [res_M4, ~] = run_method_M4_gcs_final('simple', p);
%
%   % Then compare (loads M1/M3 from saved .mat files):
%   compare_methods('simple', res_M2, res_M4);
%
%   % Optionally specify mat file paths:
%   compare_methods('simple', res_M2, res_M4, 'week3_all_results.mat', 'week2_all_results.mat');
%
% INPUTS:
%   config_name    — 'simple', 'moderate', or 'hard'
%   res_M2         — result struct returned by run_method_M2_gcs_final()
%   res_M4         — result struct returned by run_method_M4_gcs_final()
%   week3_mat_path — (optional) path to Week 3 .mat file  [default: 'week3_all_results.mat']
%   week2_mat_path — (optional) path to Week 2 .mat file  [default: 'week2_all_results.mat']
% =========================================================================

if nargin < 4 || isempty(week3_mat_path)
    week3_mat_path = 'C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\week3_all_results.mat';
end
if nargin < 5 || isempty(week2_mat_path)
    week2_mat_path = 'C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_2\week2_all_results.mat';
end

fprintf('\n');
fprintf('================================================================================\n');
fprintf('COMPARISON: M1, M2, M3, M4 — %s configuration\n', upper(config_name));
fprintf('================================================================================\n');

% ─────────────────────────────────────────────────────────────────────────
% Load M1 from Week 2 saved results
% ─────────────────────────────────────────────────────────────────────────

res_M1 = [];
if exist(week2_mat_path, 'file')
    w2 = load(week2_mat_path, 'results');
    if isfield(w2.results, config_name)
        r = w2.results.(config_name);
        res_M1.method        = 'M1 (Standard Direct Shooting)';
        res_M1.miss_distance = r.miss_cm;
        res_M1.x_land        = r.x_land;
        res_M1.t_total       = r.time_s;
        res_M1.iterations    = r.iterations;
        res_M1.energy        = r.E_total;
        res_M1.t_release     = r.t_release;
        res_M1.exit_flag     = r.exitflag;
        res_M1.n_collisions  = 0;  % Week 2 enforced via constraints
        fprintf('Loaded M1 from %s\n', week2_mat_path);
    else
        fprintf('WARNING: %s has no results for config "%s"\n', week2_mat_path, config_name);
    end
else
    fprintf('WARNING: Week 2 results file not found: %s\n', week2_mat_path);
    fprintf('         Copy your week2_all_results.mat to the working directory.\n');
end

% ─────────────────────────────────────────────────────────────────────────
% Load M3 from Week 3 saved results
% ─────────────────────────────────────────────────────────────────────────

res_M3 = [];
if exist(week3_mat_path, 'file')
    w3 = load(week3_mat_path, 'results');
    if isfield(w3.results, config_name)
        r = w3.results.(config_name);
        res_M3.method        = 'M3 (Standard Direct Collocation)';
        res_M3.miss_distance = r.miss_cm;
        res_M3.x_land        = r.x_land;
        res_M3.t_total       = r.time_s;
        res_M3.iterations    = r.iterations;
        res_M3.energy        = r.E_total;
        res_M3.t_release     = r.t_release;
        res_M3.ipopt_status  = r.ipopt_status;
        res_M3.n_collisions  = 0;
        fprintf('Loaded M3 from %s\n', week3_mat_path);
    else
        fprintf('WARNING: %s has no results for config "%s"\n', week3_mat_path, config_name);
    end
else
    fprintf('WARNING: Week 3 results file not found: %s\n', week3_mat_path);
    fprintf('         Copy your week3_all_results.mat to the working directory.\n');
end

% ─────────────────────────────────────────────────────────────────────────
% Print comparison table
% ─────────────────────────────────────────────────────────────────────────

fprintf('\n');
fprintf('%-25s | %12s | %12s | %10s | %10s | %10s\n', ...
    'Method', 'Miss (cm)', 'Time (s)', 'Iterations', 'Collisions', 'Energy (J)');
fprintf('%s\n', repmat('-', 1, 90));

print_row('M1 Standard Shooting ', res_M1);
print_row('M2 GCS+Shooting      ', res_M2);
print_row('M3 Standard Colloc   ', res_M3);
print_row('M4 GCS+Collocation   ', res_M4);

fprintf('%s\n', repmat('-', 1, 90));

% ─────────────────────────────────────────────────────────────────────────
% GCS warm-start benefit analysis
% ─────────────────────────────────────────────────────────────────────────

fprintf('\nGCS WARM-START BENEFIT\n');
fprintf('%s\n', repmat('-', 1, 60));

if ~isempty(res_M1) && ~isempty(res_M2)
    speedup   = res_M1.t_total / res_M2.t_total;
    iter_red  = (res_M1.iterations - res_M2.iterations) / max(res_M1.iterations,1) * 100;
    miss_diff = res_M1.miss_distance - res_M2.miss_distance;
    fprintf('Shooting  (M2 vs M1):  %.2fx faster | %+.1f%% iterations | %+.2f cm miss\n', ...
        speedup, -iter_red, -miss_diff);
end

if ~isempty(res_M3) && ~isempty(res_M4)
    speedup   = res_M3.t_total / res_M4.t_total;
    iter_red  = (res_M3.iterations - res_M4.iterations) / max(res_M3.iterations,1) * 100;
    miss_diff = res_M3.miss_distance - res_M4.miss_distance;
    fprintf('Colloc    (M4 vs M3):  %.2fx faster | %+.1f%% iterations | %+.2f cm miss\n', ...
        speedup, -iter_red, -miss_diff);
end

% ─────────────────────────────────────────────────────────────────────────
% Save combined results
% ─────────────────────────────────────────────────────────────────────────

combined.M1 = res_M1;
combined.M2 = res_M2;
combined.M3 = res_M3;
combined.M4 = res_M4;
combined.config = config_name;

filename = sprintf('comparison_%s.mat', config_name);
save(filename, 'combined');
fprintf('\nSaved to %s\n', filename);
fprintf('================================================================================\n');

end

% ─────────────────────────────────────────────────────────────────────────
% Helper: print one result row
% ─────────────────────────────────────────────────────────────────────────

function print_row(label, res)
if isempty(res)
    fprintf('%-25s | NOT AVAILABLE\n', label);
else
    fprintf('%-25s | %12.2f | %12.2f | %10d | %10d | %10.1f\n', ...
        label, res.miss_distance, res.t_total, res.iterations, ...
        res.n_collisions, res.energy);
end
end