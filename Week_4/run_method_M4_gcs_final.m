function [results_M4, nlp_info] = run_method_M4_gcs_final(config_name, p)
% M4: GCS-Warm-Started Direct Collocation
%
% Uses GCS to generate initial guess for direct collocation optimizer.
% Identical solver to M3 (IPOPT via CasADi), different initialization.

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('M4: GCS-Warm-Started Direct Collocation\n');
fprintf('Config: %s\n', config_name);
fprintf('%s\n', repmat('=', 1, 80));

% Load obstacles and store in p (required by build_collocation_nlp)
obstacles = load_obstacle_config(config_name, p);
p.obstacles = obstacles;

% ─────────────────────────────────────────────────────────────────────────
% GCS warm-start
% ─────────────────────────────────────────────────────────────────────────

fprintf('\nGenerating GCS warm-start initialization...\n');
tic;
[z0_gcs, gcs_meta] = gcs_to_dynamics_v2(config_name, 'direct_collocation', p);
t_gcs = toc;

fprintf('GCS init time: %.2f s\n', t_gcs);
fprintf('  Initial guess dimension: %d\n', length(z0_gcs));

% ─────────────────────────────────────────────────────────────────────────
% Build collocation NLP
% ─────────────────────────────────────────────────────────────────────────

fprintf('\nBuilding collocation NLP...\n');
nlp = build_collocation_nlp(p);

fprintf('  Variables: %d\n', nlp.n_vars);

% ─────────────────────────────────────────────────────────────────────────
% Solve with IPOPT
% ─────────────────────────────────────────────────────────────────────────

fprintf('\nRunning IPOPT optimization...\n');
tic;

% solve_collocation_nlp returns a single CasADi solution struct
sol = solve_collocation_nlp(nlp, z0_gcs, p);

t_opt = toc;

fprintf('Optimization time: %.2f s\n', t_opt);
fprintf('  Status: %s\n', sol.stats.return_status);
fprintf('  Iterations: %d\n', sol.stats.iter_count);

% ─────────────────────────────────────────────────────────────────────────
% Extract solution
% unpack_collocation_solution expects numeric z_opt, not CasADi sol struct
% ─────────────────────────────────────────────────────────────────────────

% Extract solution (get each variable separately and concatenate)
q_opt = full(sol.value(nlp.q));
qdot_opt = full(sol.value(nlp.qdot));
tau_opt = full(sol.value(nlp.tau));
t_release_opt = full(sol.value(nlp.t_release));

% Concatenate into single vector z_opt
z_opt_numeric = [q_opt(:); qdot_opt(:); tau_opt(:); t_release_opt];
traj = unpack_collocation_solution(z_opt_numeric, nlp, p);

x_land     = traj.x_land;
miss       = abs(x_land - p.task.d) * 100;

fprintf('\nResult: %.2f cm miss, t_release = %.3f s\n', miss, traj.t_release);

% Collision count — build link query struct for each time step
n_coll = 0;
for i = 1:length(traj.t)
    [jpos, ~] = forward_kinematics(traj.q(:, i), p);
    arm_collides = false;
    for link = 1:p.N
        query.type = 'link';
        query.p1   = jpos(:, link);
        query.p2   = jpos(:, link + 1);
        [coll, ~]  = check_collision(query, obstacles, p);
        if coll
            arm_collides = true;
            break;
        end
    end
    if arm_collides
        n_coll = n_coll + 1;
    end
end
fprintf('  Collisions: %d\n', n_coll);

% ─────────────────────────────────────────────────────────────────────────
% Package results
% ─────────────────────────────────────────────────────────────────────────

results_M4.method         = 'M4 (GCS-Warm-Started Direct Collocation)';
results_M4.config         = config_name;
results_M4.miss_distance  = miss;
results_M4.x_land         = x_land;
results_M4.t_total        = t_gcs + t_opt;
results_M4.t_gcs          = t_gcs;
results_M4.t_optimization = t_opt;
results_M4.iterations     = sol.stats.iter_count;
results_M4.ipopt_status   = sol.stats.return_status;
results_M4.n_collisions   = n_coll;
results_M4.collision_free = (n_coll == 0);
results_M4.trajectory     = traj;

% Energy
E  = 0;
dt = traj.t_release / length(traj.t);
for i = 1:length(traj.t)
    power = traj.tau(:, i)' * traj.qdot(:, i);
    if power > 0
        E = E + power * dt;
    end
end
results_M4.energy = E;

nlp_info.ipopt_status   = sol.stats.return_status;
nlp_info.iterations     = sol.stats.iter_count;
nlp_info.gcs_metadata   = gcs_meta;

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('M4 Complete\n');
fprintf('%s\n', repmat('=', 1, 80));

end