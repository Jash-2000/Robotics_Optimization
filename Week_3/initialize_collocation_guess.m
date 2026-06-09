function z0 = initialize_collocation_guess(nlp, p)
% initialize_collocation_guess.m
% =========================================================================
% Creates initial guess for direct collocation with COLLISION AWARENESS.
% 
% Strategy: Two-phase motion pattern
%   Phase 1 (0 to t_mid): BACKWARD SWING
%      - Negative shoulder torque rotates arm downward/backward
%      - Builds angular momentum
%      - Moves AWAY from obstacles (up and back)
%   Phase 2 (t_mid to t_release): FORWARD SWING
%      - Strong positive torques accelerate arm forward
%      - Releases object with high velocity
%      - Smooth ramping to avoid jerky motion
% 
% This two-phase pattern is much more likely to be collision-free than
% a simple forward swing, because the backward phase moves the arm away
% from obstacles initially.
% =========================================================================

M = nlp.M;
N = nlp.N;

fprintf('    Generating collision-aware backward-swing initial guess...\n');

% Release time guess
t_release_init = 1.3;  % seconds (slightly longer for backward swing)

%% ── Define two-phase torque pattern ───────────────────────────────────────
% Transition at midpoint
t_mid = 0.65;  % Halfway through

% Create torque function handle
tau_func = @(t, q, qdot) torque_pattern(t, q, qdot, t_mid, t_release_init);

%% ── Integrate ODE with two-phase torques ──────────────────────────────────
fprintf('    Simulating backward-swing trajectory...\n');

y0 = [p.q0; p.qdot0];
tspan = linspace(0, t_release_init, M);

ode_func = @(t, y) arm_ode(t, y, tau_func, p);
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);

[t_vec, y_vec] = ode45(ode_func, tspan, y0, ode_opts);

%% ── Extract state and control at collocation points ──────────────────────
q_init = zeros(N, M);
qdot_init = zeros(N, M);
tau_init = zeros(N, M);

for k = 1:M
    t_k = t_vec(k);
    q_init(:, k) = y_vec(k, 1:N)';
    qdot_init(:, k) = y_vec(k, N+1:end)';
    tau_init(:, k) = tau_func(t_k, q_init(:, k), qdot_init(:, k));
end

%% ── Package into decision variable vector ────────────────────────────────
z0 = [q_init(:); qdot_init(:); tau_init(:); t_release_init];

fprintf('    Initial guess statistics:\n');
fprintf('      q range: [%.2f, %.2f] rad (backward swing range)\n', min(q_init(:)), max(q_init(:)));
fprintf('      qdot range: [%.2f, %.2f] rad/s\n', min(qdot_init(:)), max(qdot_init(:)));
fprintf('      tau range: [%.2f, %.2f] N·m (backward→forward)\n', min(tau_init(:)), max(tau_init(:)));
fprintf('      t_release: %.2f s\n', t_release_init);
fprintf('    Strategy: Backward swing [0, %.2f]s → Forward swing [%.2f, %.2f]s\n', ...
    t_mid, t_mid, t_release_init);

end

%% ═══════════════════════════════════════════════════════════════════════════
%% HELPER: Two-phase torque pattern (backward → forward)
%% ═══════════════════════════════════════════════════════════════════════════

function tau = torque_pattern(t, q, qdot, t_mid, t_release)
% Two-phase torque pattern:
%   Phase 1 (0 ≤ t ≤ t_mid): Backward swing (negative shoulder torque)
%   Phase 2 (t_mid < t ≤ t_release): Forward swing (positive ramp)

    % Smooth transition from Phase 1 to Phase 2
    transition_width = 0.1;  % Width of transition region [seconds]
    
    % Sigmoid-like transition: 0 at (t_mid - width), 1 at (t_mid + width)
    if t < (t_mid - transition_width)
        phase = 0;  % Pure backward
    elseif t > (t_mid + transition_width)
        phase = 1;  % Pure forward
    else
        % Smooth interpolation
        s = (t - (t_mid - transition_width)) / (2 * transition_width);
        phase = s * s * (3 - 2*s);  % Hermite smoothstep
    end
    
    %% ── PHASE 1: Backward swing ──────────────────────────────────────────
    % Shoulder (joint 1): Strong negative (rotate DOWN/BACK away from obstacles)
    % Other joints: Damped oscillations for stability
    
    A_back = [-4.0;  -1.0;  -0.5;  -0.3];  % Negative amplitudes
    omega_back = [-0.8; -0.3; -0.35; -0.4];
    phi_back = [0; -0.3; -0.6; 0];
    
    tau_backward = A_back .* sin(omega_back * t + phi_back);
    
    % Damping: Add velocity-dependent damping to stabilize
    tau_backward = tau_backward - 0.5 * qdot;
    
    %% ── PHASE 2: Forward swing ───────────────────────────────────────────
    % Linear ramp from 0 at t_mid to maximum at t_release
    
    t_forward = t - t_mid;
    if t_forward <= 0
        tau_forward = zeros(4, 1);
    else
        % Normalized ramp [0, 1]
        ramp = t_forward / (t_release - t_mid);
        ramp = max(0, min(1, ramp));
        
        % Strong accelerating torques (ramped)
        A_forward = [9.0; 2.0; 1.2; 1.0] .* ramp;
        omega_forward = [1.5; 0.6; 0.5; 0.4];
        phi_forward = [pi/2; -0.2; -0.5; 0];
        
        tau_forward = A_forward .* sin(omega_forward * t + phi_forward);
    end
    
    %% ── Blend between phases ─────────────────────────────────────────────
    % phase = 0 → tau_backward
    % phase = 1 → tau_forward
    tau = (1 - phase) * tau_backward + phase * tau_forward;
    
end