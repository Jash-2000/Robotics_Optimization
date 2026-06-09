function compare_methods_all(config_name, res_M5, res_M6, varargin)
% compare_methods_all.m
% =========================================================================
% Build a 6-method comparison table for a single obstacle configuration.
%
% Loads M1–M4 from saved .mat files; takes M5/M6 as live arguments.
%
% USAGE:
%   p = params();
%   [res_M5, ~] = run_method_M5_scp('simple', p);
%   [res_M6, ~] = run_method_M6_gcs_scp('simple', p);
%   compare_methods_all('simple', res_M5, res_M6);
%
%   % Optionally pass paths to Week 2/3/4 .mat files:
%   compare_methods_all('simple', res_M5, res_M6, ...
%       'comparison_simple.mat', 'week2_all_results.mat', 'week3_all_results.mat');
%
% INPUTS:
%   config_name    — 'simple', 'moderate', or 'hard'
%   res_M5         — result struct from run_method_M5_scp
%   res_M6         — result struct from run_method_M6_gcs_scp
%   varargin{1}    — (opt) comparison_<config>.mat path
%   varargin{2}    — (opt) week2_all_results.mat path
%   varargin{3}    — (opt) week3_all_results.mat path
% =========================================================================

% ── Parse optional paths ─────────────────────────────────────────────
if nargin >= 4 && ~isempty(varargin{1})
    comp_path = varargin{1};
else
    comp_path = sprintf('comparison_%s.mat', config_name);
end
if nargin >= 5, w2_path = varargin{2}; else, w2_path = 'week2_all_results.mat'; end
if nargin >= 6, w3_path = varargin{3}; else, w3_path = 'week3_all_results.mat'; end

fprintf('\n');
fprintf('════════════════════════════════════════════════════════════════════════\n');
fprintf('  6-METHOD COMPARISON — %s configuration\n', upper(config_name));
fprintf('════════════════════════════════════════════════════════════════════════\n');

% ── Load M1–M4 from saved results ───────────────────────────────────
res = cell(6,1);     % {M1, M2, M3, M4, M5, M6}
labels = {'M1 (Std Shooting)', 'M2 (GCS Shooting)', ...
          'M3 (Std Collocation)', 'M4 (GCS Collocation)', ...
          'M5 (Std SCP)', 'M6 (GCS SCP)'};

% Try loading from comparison_<config>.mat first (has M1–M4)
if exist(comp_path, 'file')
    data = load(comp_path);
    if isfield(data, 'combined')
        c = data.combined;
        if isfield(c, 'M1'), res{1} = extract_result(c.M1, 'M1'); end
        if isfield(c, 'M2'), res{2} = extract_result(c.M2, 'M2'); end
        if isfield(c, 'M3'), res{3} = extract_result(c.M3, 'M3'); end
        if isfield(c, 'M4'), res{4} = extract_result(c.M4, 'M4'); end
        fprintf('  Loaded M1–M4 from %s\n', comp_path);
    end
else
    % Fall back to separate week2/week3 files
    if exist(w2_path, 'file')
        w2 = load(w2_path, 'results');
        if isfield(w2.results, config_name)
            r = w2.results.(config_name);
            res{1} = struct('miss_cm', r.miss_cm, 'time_s', r.time_s, ...
                'iterations', r.iterations, 'energy', r.E_total, ...
                'collisions', 0, 't_release', r.t_release, 'x_land', r.x_land);
        end
    end
    if exist(w3_path, 'file')
        w3 = load(w3_path, 'results');
        if isfield(w3.results, config_name)
            r = w3.results.(config_name);
            res{3} = struct('miss_cm', r.miss_cm, 'time_s', r.time_s, ...
                'iterations', r.iterations, 'energy', r.E_total, ...
                'collisions', 0, 't_release', r.t_release, 'x_land', r.x_land);
        end
    end
end

% M5 and M6 from live results
res{5} = struct('miss_cm', res_M5.miss_distance, 'time_s', res_M5.t_total, ...
    'iterations', res_M5.iterations, 'energy', res_M5.energy, ...
    'collisions', res_M5.n_collisions, 't_release', res_M5.t_release, ...
    'x_land', res_M5.x_land);
res{6} = struct('miss_cm', res_M6.miss_distance, 'time_s', res_M6.t_total, ...
    'iterations', res_M6.iterations, 'energy', res_M6.energy, ...
    'collisions', res_M6.n_collisions, 't_release', res_M6.t_release, ...
    'x_land', res_M6.x_land);

% ── Print comparison table ───────────────────────────────────────────
fprintf('\n');
fprintf('  %-26s  %8s  %8s  %6s  %8s  %6s  %8s\n', ...
    'Method', 'Miss(cm)', 'Time(s)', 'Iters', 'Energy', 'Coll', 'x_land');
fprintf('  %s\n', repmat('─', 1, 82));

for i = 1:6
    if isempty(res{i})
        fprintf('  %-26s  %8s  %8s  %6s  %8s  %6s  %8s\n', ...
            labels{i}, '—', '—', '—', '—', '—', '—');
    else
        r = res{i};
        fprintf('  %-26s  %8.1f  %8.2f  %6d  %8.1f  %6d  %8.3f\n', ...
            labels{i}, r.miss_cm, r.time_s, r.iterations, ...
            r.energy, r.collisions, r.x_land);
    end
end
fprintf('  %s\n', repmat('─', 1, 82));

% ── Speedup analysis ─────────────────────────────────────────────────
fprintf('\n  Warm-start effects:\n');
pairs = {1,2,'M1→M2'; 3,4,'M3→M4'; 5,6,'M5→M6'};
for k = 1:size(pairs,1)
    i_std = pairs{k,1};
    i_gcs = pairs{k,2};
    label = pairs{k,3};
    if ~isempty(res{i_std}) && ~isempty(res{i_gcs})
        speedup = res{i_std}.time_s / max(res{i_gcs}.time_s, 0.01);
        miss_delta = res{i_gcs}.miss_cm - res{i_std}.miss_cm;
        fprintf('    %s:  time speedup = %.2f×,  miss Δ = %+.1f cm\n', ...
            label, speedup, miss_delta);
    end
end

% ── SCP vs Collocation comparison ────────────────────────────────────
fprintf('\n  SCP vs Collocation:\n');
if ~isempty(res{3}) && ~isempty(res{5})
    fprintf('    M3 vs M5:  miss Δ = %+.1f cm,  time Δ = %+.1f s\n', ...
        res{5}.miss_cm - res{3}.miss_cm, res{5}.time_s - res{3}.time_s);
end
if ~isempty(res{4}) && ~isempty(res{6})
    fprintf('    M4 vs M6:  miss Δ = %+.1f cm,  time Δ = %+.1f s\n', ...
        res{6}.miss_cm - res{4}.miss_cm, res{6}.time_s - res{4}.time_s);
end

fprintf('\n');

% ── Save combined results ────────────────────────────────────────────
combined_all.config = config_name;
combined_all.labels = labels;
combined_all.results = res;
combined_all.res_M5 = res_M5;
combined_all.res_M6 = res_M6;

save_name = sprintf('comparison_all_%s.mat', config_name);
save(save_name, 'combined_all');
fprintf('  Results saved to %s\n\n', save_name);

end


% ═════════════════════════════════════════════════════════════════════════
%  HELPER: Extract a uniform result struct from loaded .mat data
% ═════════════════════════════════════════════════════════════════════════
function r = extract_result(s, method_id)
    r = struct();
    % Normalise field names across M1–M4 saved formats
    if isfield(s, 'miss_distance')
        r.miss_cm = s.miss_distance;
    elseif isfield(s, 'miss_cm')
        r.miss_cm = s.miss_cm;
    else
        r.miss_cm = NaN;
    end

    if isfield(s, 't_total')
        r.time_s = s.t_total;
    elseif isfield(s, 'time_s')
        r.time_s = s.time_s;
    else
        r.time_s = NaN;
    end

    if isfield(s, 'iterations')
        r.iterations = s.iterations;
    else
        r.iterations = 0;
    end

    if isfield(s, 'energy')
        r.energy = s.energy;
    elseif isfield(s, 'E_total')
        r.energy = s.E_total;
    else
        r.energy = NaN;
    end

    if isfield(s, 'n_collisions')
        r.collisions = s.n_collisions;
    else
        r.collisions = 0;
    end

    if isfield(s, 't_release')
        r.t_release = s.t_release;
    else
        r.t_release = NaN;
    end

    if isfield(s, 'x_land')
        r.x_land = s.x_land;
    else
        r.x_land = NaN;
    end
end