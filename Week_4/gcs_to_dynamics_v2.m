function [z0, metadata] = gcs_to_dynamics_v2(config_name, method_type, p)
% gcs_to_dynamics_v2.m
% =========================================================================
% GCS Repair Bridge v2: Convert geometric paths to dynamically-feasible
% initial guesses with intelligent IK solving and fallback.
%
% KEY INSIGHT: Not all GCS paths will have perfect IK solutions.
% We try multiple approaches and pick the best one that works.
%
% OUTPUTS:
%   z0       — Initial guess in correct format for method
%   metadata — Diagnostic info
% =========================================================================

fprintf('\n%s\n', repmat('═', 1, 70));
fprintf('GCS Repair Bridge v2 (Robust IK + Fallback)\n');
fprintf('Config: %s, Method: %s\n', config_name, method_type);
fprintf('%s\n', repmat('═', 1, 70));

% ────────────────────────────────────────────────────────────────────────
% STEP 1-2: Solve GCS geometric path planning
% ────────────────────────────────────────────────────────────────────────

fprintf('\n[STEP 1-2] GCS geometric path planning\n');
[paths, regions, region_path] = gcs_solver_v2(config_name, p);

fprintf('Generated %d candidate workspace paths\n', length(paths));

% ────────────────────────────────────────────────────────────────────────
% STEP 3-6: Try each GCS path with robust multi-strategy IK
% ────────────────────────────────────────────────────────────────────────

fprintf('\n[STEP 3-6] Converting paths to joint trajectories (robust IK)\n');

best_trajectory = [];
best_path_id = [];
best_error = inf;

for path_id = 1:length(paths)
    fprintf('\nCandidate path %d: ', path_id);
    
    path_workspace = paths{path_id};
    
    % Try IK with multiple strategies
    [q_traj, ik_success, ik_error] = robust_ik_solver(path_workspace, p);
    
    if ~ik_success
        fprintf('IK failed (mean error: %.2e)\n', ik_error);
        continue;
    end
    
    % Skip collision check for now — will validate in trajectory later
    
    fprintf('IK success ✓\n');
    
    % Check for joint smoothness
    max_jump = 0;
    for i = 2:size(q_traj, 2)
        jump = max(abs(q_traj(:, i) - q_traj(:, i-1)));
        max_jump = max(max_jump, jump);
    end
    
    fprintf('  Joint smoothness: max jump = %.2f rad\n', max_jump);
    
    % Keep best path
    if ik_error < best_error
        best_error = ik_error;
        best_trajectory = q_traj;
        best_path_id = path_id;
    end
end

if isempty(best_trajectory)
    error('All GCS paths failed IK validation. Try different region definitions.');
end

fprintf('\n✓ Selected path %d (IK error: %.2e)\n', best_path_id, best_error);

% ────────────────────────────────────────────────────────────────────────
% STEP 7: Time-scale to satisfy torque limits
% ────────────────────────────────────────────────────────────────────────

fprintf('\n[STEP 7] Time-scaling for torque limits\n');

[q_traj, qdot_traj, tau_traj, t_release] = time_scale_trajectory(best_trajectory, p);

fprintf('Time-scaled trajectory: t_release = %.3f s\n', t_release);
fprintf('Torque range: [%.2f, %.2f] N·m (limit: %.2f)\n', ...
    min(tau_traj(:)), max(tau_traj(:)), max(p.lim.tau_max));

% ────────────────────────────────────────────────────────────────────────
% STEP 8: Append throwing swing
% ────────────────────────────────────────────────────────────────────────

fprintf('\n[STEP 8] Appending throwing swing\n');

[q_full, qdot_full, tau_full] = append_throwing_swing(q_traj, qdot_traj, tau_traj, t_release, p);

fprintf('Complete trajectory: %d time points, t_release = %.3f s\n', ...
    size(q_full, 2), t_release);

% ────────────────────────────────────────────────────────────────────────
% Package into method-specific initial guess
% ────────────────────────────────────────────────────────────────────────

fprintf('\n[PACKAGING] Formatting for %s\n', method_type);

if strcmp(method_type, 'direct_shooting')
    % Direct shooting: piecewise-constant torques
    % Format expected by unpack_torques: [tau(t1), tau(t2), ..., tau(tM), t_release]
    % where each tau(tk) is [tau1, tau2, tau3, tau4] for 4 joints
    
    % Initialize p.opt.M if not set
    if ~isfield(p, 'opt') || ~isfield(p.opt, 'M')
        p.opt.M = 60;
    end
    M = p.opt.M;
    
    % Resample tau_full to M piecewise-constant points
    t_fine = linspace(0, t_release, size(tau_full, 2));
    t_coll = linspace(0, t_release, M);
    
    % Resample each joint's torque
    tau_coll = zeros(p.N, M);
    for joint = 1:p.N
        tau_coll(joint, :) = interp1(t_fine, tau_full(joint, :), t_coll, 'nearest');
    end
    
    % Pack as column vector: [tau1(t1), tau2(t1), tau3(t1), tau4(t1), tau1(t2), ..., t_release]
    z0 = [tau_coll(:); t_release];
    fprintf('✓ Shooting z0: %d variables (4*%d + 1)\n', length(z0), M);
    
elseif strcmp(method_type, 'direct_collocation')
    % Initialize p.opt.M if not set
    if ~isfield(p, 'opt') || ~isfield(p.opt, 'M')
        p.opt.M = 60;
    end
    
    % Resample to M collocation points
    M = p.opt.M;
    t_coll = linspace(0, t_release, M);
    t_fine = linspace(0, t_release, size(q_full, 2));
    
    q_coll = interp1(t_fine, q_full', t_coll, 'spline')';
    qdot_coll = interp1(t_fine, qdot_full', t_coll, 'spline')';
    tau_coll = interp1(t_fine, tau_full', t_coll, 'spline')';
    
    z0 = [q_coll(:); qdot_coll(:); tau_coll(:); t_release];
    fprintf('✓ Collocation z0: %d variables\n', length(z0));
end

% ────────────────────────────────────────────────────────────────────────
% Metadata
% ────────────────────────────────────────────────────────────────────────

metadata.config = config_name;
metadata.method = method_type;
metadata.gcs_path_id = best_path_id;
metadata.t_release = t_release;
metadata.q_full = q_full;
metadata.qdot_full = qdot_full;
metadata.tau_full = tau_full;
metadata.tau_max = max(abs(tau_full(:)));
metadata.region_path = region_path;

fprintf('\n%s\n', repmat('═', 1, 70));
fprintf('✓ GCS repair bridge complete\n');
fprintf('%s\n', repmat('═', 1, 70));

end

% ═══════════════════════════════════════════════════════════════════════
% ROBUST IK SOLVER
% ═══════════════════════════════════════════════════════════════════════

function [q_traj, success, mean_error] = robust_ik_solver(path_workspace, p)
% Solve IK using Jacobian pseudoinverse (fast) with fmincon fallback

q_traj = zeros(p.N, size(path_workspace, 1));
q_current = p.q0;
errors = [];
base = [0; p.y0];
L_total = sum(p.l);

for i = 1:size(path_workspace, 1)
    target = path_workspace(i, :)';
    
    % Skip if target is unreachable
    dist_from_base = norm(target - base);
    if dist_from_base > L_total * 0.98
        % Clamp target to reachable boundary
        target = base + (target - base) * (L_total * 0.95 / dist_from_base);
    end
    
    % Method 1: Jacobian pseudoinverse IK (fast, iterative)
    q_ik = q_current;
    for iter = 1:200
        [jpos, ~] = forward_kinematics(q_ik, p);
        ee = jpos(:, end);
        err_vec = target - ee;
        err_norm = norm(err_vec);
        
        if err_norm < 1e-4  % 0.1mm — converged
            break;
        end
        
        % Numerical Jacobian (2×4)
        J = zeros(2, p.N);
        dq = 1e-6;
        for j = 1:p.N
            q_plus = q_ik;
            q_plus(j) = q_plus(j) + dq;
            [jpos_plus, ~] = forward_kinematics(q_plus, p);
            J(:, j) = (jpos_plus(:, end) - ee) / dq;
        end
        
        % Damped pseudoinverse step
        lambda = 0.01;
        dq_step = J' * ((J * J' + lambda^2 * eye(2)) \ err_vec);
        q_ik = q_ik + 0.5 * dq_step;  % Step size 0.5 for stability
    end
    
    [jpos_final, ~] = forward_kinematics(q_ik, p);
    best_err = norm(jpos_final(:, end) - target);
    best_q = q_ik;
    
    % Method 2: If Jacobian IK didn't converge, try fmincon
    if best_err > 0.01
        for seed = 1:3
            if seed == 1
                q_init = q_current;
            elseif seed == 2
                q_init = q_ik;  % Start from Jacobian result
            else
                q_init = q_current + 0.3 * randn(p.N, 1);
            end
            
            opts = optimoptions('fmincon', 'Display', 'off', ...
                'MaxIterations', 500, 'TolFun', 1e-8, 'Algorithm', 'sqp');
            
            [q_test, err, ~] = fmincon(@(q) norm(fk_endpoint(q, p) - target), ...
                q_init, [], [], [], [], ...
                -pi*ones(p.N,1), pi*ones(p.N,1), [], opts);
            
            if err < best_err
                best_q = q_test;
                best_err = err;
            end
        end
    end
    
    % Accept if error is within 5cm (relaxed — this is just an initial guess)
    if best_err < 0.05
        q_traj(:, i) = best_q;
        q_current = best_q;
        errors = [errors, best_err];
    else
        fprintf('    IK failed at wp %d: target=[%.3f,%.3f], err=%.4f m, dist=%.3f m\n', ...
            i, target(1), target(2), best_err, dist_from_base);
        success = false;
        mean_error = best_err;
        return;
    end
end

success = true;
mean_error = mean(errors);

end

function p_ee = fk_endpoint(q, p)
[jpos, ~] = forward_kinematics(q, p);
p_ee = jpos(:, end);
end

% ═══════════════════════════════════════════════════════════════════════
% TIME SCALING
% ═══════════════════════════════════════════════════════════════════════

function [q_traj, qdot_traj, tau_traj, t_release] = time_scale_trajectory(q_traj, p)

% Start with 1.0s
t_release = 1.0;
N_points = size(q_traj, 2);
t_vec = linspace(0, t_release, N_points);

% Compute velocities
qdot_traj = zeros(p.N, N_points);
for i = 2:N_points-1
    dt = t_vec(i+1) - t_vec(i-1);
    qdot_traj(:, i) = (q_traj(:, i+1) - q_traj(:, i-1)) / dt;
end

% Compute accelerations
qddot_traj = zeros(p.N, N_points);
for i = 2:N_points-1
    dt = t_vec(i+1) - t_vec(i-1);
    qddot_traj(:, i) = (qdot_traj(:, i+1) - qdot_traj(:, i-1)) / dt;
end

% Compute torques
tau_traj = zeros(p.N, N_points);
for i = 1:N_points
    M_q = M_func(q_traj(:, i));
    C_q = C_func(q_traj(:, i), qdot_traj(:, i));
    G_q = G_func(q_traj(:, i));
    tau_traj(:, i) = M_q * qddot_traj(:, i) + C_q * qdot_traj(:, i) + G_q;
end

% Scale if needed
tau_max_actual = max(abs(tau_traj(:)));
tau_limit = max(p.lim.tau_max);  % Most restrictive limit
if tau_max_actual > tau_limit
    scale = sqrt(tau_max_actual / tau_limit);
    t_release = t_release * scale;
    qdot_traj = qdot_traj / scale;
    qddot_traj = qddot_traj / (scale^2);
    tau_traj = tau_traj / (scale^2);
end

end

% ═══════════════════════════════════════════════════════════════════════
% APPEND THROWING SWING
% ═══════════════════════════════════════════════════════════════════════

function [q_full, qdot_full, tau_full] = append_throwing_swing(q_traj, qdot_traj, tau_traj, t_release, p)

% Append ramped throwing motion
t_throw = linspace(t_release, 1.5*t_release, 15);
dt = mean(diff(t_throw));

q_curr = q_traj(:, end);
qdot_curr = zeros(p.N, 1);

q_throw = [];
qdot_throw = [];
tau_throw = [];

for i = 1:length(t_throw)
    t_i = t_throw(i);
    phase = (t_i - t_release) / (0.5 * t_release);  % 0 to 1
    
    % Ramp-up then ramp-down torque
    if phase < 0.5
        ramp = phase * 2;
    else
        ramp = (1 - phase) * 2;
    end
    
    tau_i = ramp * [10; 2; 1.5; 1];
    
    % Euler step
    M_q = M_func(q_curr);
    C_q = C_func(q_curr, qdot_curr);
    G_q = G_func(q_curr);
    qddot_curr = M_q \ (tau_i - C_q * qdot_curr - G_q);
    
    q_curr = q_curr + dt * qdot_curr;
    qdot_curr = qdot_curr + dt * qddot_curr;
    
    q_throw = [q_throw, q_curr];
    qdot_throw = [qdot_throw, qdot_curr];
    tau_throw = [tau_throw, tau_i];
end

q_full = [q_traj, q_throw];
qdot_full = [qdot_traj, qdot_throw];
tau_full = [tau_traj, tau_throw];

end

% ═══════════════════════════════════════════════════════════════════════
% FIT TORQUE HARMONICS
% ═══════════════════════════════════════════════════════════════════════

function tau_params = fit_torque_harmonics(tau_traj, t_release, p)

tau_params = [];

for joint = 1:p.N
    tau_j = tau_traj(joint, :);
    
    A_j = max(abs(tau_j));
    
    % Frequency estimate from period
    t_vec = linspace(0, t_release, length(tau_j));
    omega_j = 2 * pi / (2 * t_release);  % ~half-period oscillation
    
    % Phase fitting
    X = [sin(omega_j * t_vec); cos(omega_j * t_vec)]';
    coeffs = X \ tau_j(:);
    phi_j = atan2(coeffs(2), coeffs(1));
    
    tau_params = [tau_params; A_j; omega_j; phi_j];
end

end