function validate_energy_conservation(p)
% validate_energy_conservation.m
% =========================================================================
% Validates that the derived Lagrangian dynamics conserve energy when no
% external torques are applied (τ = 0). This is a critical sanity check
% for the symbolic derivation in derive_dynamics.m.
%
% TEST:
%   Run ODE with τ = 0 for 2 seconds
%   Compute E(t) = T(q,qdot) + V(q) at each timestep
%   Verify E(t) ≈ E(0) to within numerical tolerance (< 0.1% drift)
%
% INPUTS:
%   p  – parameter struct from params.m
%
% OUTPUT:
%   Prints PASS/FAIL and shows energy plot
% =========================================================================

fprintf('\n=== Energy Conservation Validation ===\n');
fprintf('  Running unactuated dynamics (tau = 0) ...\n');

%% ── Run ODE with zero torque ─────────────────────────────────────────────
tau_func = @(t, q, qdot) zeros(p.N, 1);   % no actuation

% Start from a non-trivial state (arm bent, moving)
q_init = p.q0 + [0.3; -0.2; 0.1; -0.05];   % bent configuration
qdot_init = [0.5; -0.3; 0.2; 0.1];         % small velocities
x0 = [q_init; qdot_init];

t_span = [0, 2.0];   % 2 second test
t_eval = (0 : 0.01 : 2.0)';

[t_sol, x_sol] = ode45(...
    @(t,x) arm_ode(t, x, tau_func, p), ...
    t_eval, x0, p.sim.ode_opts);

fprintf('    ODE integrated: %d steps\n', length(t_sol));

%% ── Compute energy at each timestep ──────────────────────────────────────
n_steps = length(t_sol);
E_vec   = zeros(n_steps, 1);
T_vec   = zeros(n_steps, 1);
V_vec   = zeros(n_steps, 1);

for k = 1:n_steps
    q_k    = x_sol(k, 1:p.N)';
    qdot_k = x_sol(k, p.N+1:2*p.N)';
    
    T_vec(k) = T_func(q_k, qdot_k);   % kinetic energy
    V_vec(k) = V_func(q_k);           % potential energy
    E_vec(k) = T_vec(k) + V_vec(k);   % total energy
end

%% ── Check conservation ───────────────────────────────────────────────────
E_initial = E_vec(1);
E_drift   = E_vec - E_initial;
max_drift = max(abs(E_drift));
rel_drift = max_drift / abs(E_initial);

fprintf('    Initial energy E(0)  : %.6f J\n', E_initial);
fprintf('    Max absolute drift   : %.6e J\n', max_drift);
fprintf('    Max relative drift   : %.4f %%\n', rel_drift * 100);

PASS_THRESHOLD = 0.001;   % 0.1% tolerance

if rel_drift < PASS_THRESHOLD
    fprintf('    Status: PASS ✓ (drift < %.1f%%)\n', PASS_THRESHOLD*100);
    status_str = 'PASS ✓';
    status_col = [0.1 0.7 0.1];
else
    fprintf('    Status: FAIL ✗ (drift > %.1f%%)\n', PASS_THRESHOLD*100);
    fprintf('    WARNING: Energy is not conserved. Check derive_dynamics.m.\n');
    status_str = 'FAIL ✗';
    status_col = [0.9 0.1 0.1];
end

fprintf('======================================\n\n');

%% ── Plot energy over time ────────────────────────────────────────────────
figure('Name', 'Energy Conservation Validation', ...
    'Position', [200, 200, 900, 500], 'Color', 'white');

subplot(2,1,1);
plot(t_sol, T_vec, 'LineWidth', 2, 'Color', [0.85 0.33 0.10]); hold on;
plot(t_sol, V_vec, 'LineWidth', 2, 'Color', [0.20 0.45 0.70]);
plot(t_sol, E_vec, 'LineWidth', 2.5, 'Color', [0.2 0.2 0.2], 'LineStyle', '--');
xlabel('Time [s]', 'FontSize', 11);
ylabel('Energy [J]', 'FontSize', 11);
title('Energy Components (Zero Torque)', 'FontSize', 12, 'FontWeight', 'bold');
legend({'Kinetic T', 'Potential V', 'Total E'}, 'Location', 'best', 'FontSize', 10);
grid on;

subplot(2,1,2);
plot(t_sol, E_drift * 1e3, 'LineWidth', 2, 'Color', status_col);
xlabel('Time [s]', 'FontSize', 11);
ylabel('Energy Drift [mJ]', 'FontSize', 11);
title(sprintf('Energy Drift from E(0)  |  Status: %s', status_str), ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', status_col);
grid on;
yline(0, 'k--', 'LineWidth', 1);

drawnow;

end
