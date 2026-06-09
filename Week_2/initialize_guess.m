function z0 = initialize_guess(p)
% initialize_guess.m
% =========================================================================
% Creates initial guess for decision variables based on Week 1 fixed
% sinusoidal torques. Samples the sinusoidal profile at M time points.
%
% NEW: Includes release time as last decision variable
%
% INPUTS:
%   p – parameter struct from params.m (contains p.opt.M)
%
% OUTPUT:
%   z0 – [4M+1×1] initial guess: [torques(1:4M); t_release]
%
% WEEK 1 BASELINE TORQUES:
%   τᵢ(t) = Aᵢ * sin(ωᵢ*t + φᵢ)
%   where:
%     A = [15.0, 8.0, 4.0, 0.15] N·m (amplitudes)
%     ω = [0.5, 0.30, 0.35, 0.4] rad/s (frequencies)
%     φ = [0, -0.3, -0.6, 0] rad (phases)
%
% STRATEGY:
%   - Sample baseline torques at M uniformly spaced time points
%   - Add small random noise (5% of amplitude) to avoid singular Jacobian
%   - Ensure initial guess respects torque limits
%   - Initial release time: 1.2 s (from Week 1 baseline)
% =========================================================================

M = p.opt.M;
N = p.N;

% Initial release time guess (from Week 1)
t_release_init = 1.5;  % seconds

% Week 1 baseline parameters (from main_week1.m)
A   = [1.5;  8;  0.4;  0.5];      % amplitudes [N·m]
omega = [1; 0.30; 0.35; 0.4];    % frequencies [rad/s]
phi = [0; -0.3; -0.6; 0];          % phases [rad]

% Time points: centers of M intervals
dt = t_release_init / M;
t_centers = ((1:M) - 0.5) * dt;    % [dt/2, 3dt/2, ..., (M-0.5)*dt]

% Evaluate baseline sinusoidal torques at each time point
z0_torques = zeros(N*M, 1);
% In initialize_guess.m:
for k = 1:M
    % Average the sinusoid over the interval
    t_start = (k-1) * dt;
    t_end = k * dt;
    t_samples = linspace(t_start, t_end, 10);  % 10 samples per interval
    
    % Average torque over the interval
    tau_avg = zeros(N, 1);
    for j = 1:length(t_samples)
        tau_avg = tau_avg + A .* sin(omega * t_samples(j) + phi);
    end
    tau_avg = tau_avg / length(t_samples);
    
    % No noise for now (you already set it to 0)
    tau_noisy = tau_avg;
    
    % Clamp to limits
    tau_noisy = max(p.lim.tau_min, min(p.lim.tau_max, tau_noisy));
    
    % Store
    idx_start = N*(k-1) + 1;
    idx_end = N*k;
    z0_torques(idx_start:idx_end) = tau_noisy;
end
% Append release time as last element
z0 = [z0_torques; t_release_init];

fprintf('  Initial guess created: z0 ∈ R^%d (%d torques + 1 release time)\n', ...
    length(z0), N*M);
fprintf('  Baseline: sinusoidal torques from Week 1, sampled + noise\n');
fprintf('  Initial release time: %.2f s\n', t_release_init);

end