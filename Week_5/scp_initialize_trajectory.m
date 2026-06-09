function [x0, u0, t_f0] = scp_initialize_trajectory(p, scp, method)
% scp_initialize_trajectory.m
% =========================================================================
% Create an initial reference trajectory for the SCP loop.
%
% Uses RK4 integration for the rollout so the trajectory is smooth and
% stable. The QP equality constraints still use Forward Euler, but we
% handle the consistency issue by setting beq from the actual rollout
% defects in scp_build_subproblem (the linearisation is around x_bar,
% so the defect gets absorbed into the RHS).
%
% INPUTS:
%   p      — parameter struct
%   scp    — SCP parameter struct
%   method — 'standard' or 'gcs'
%
% OUTPUTS:
%   x0   — [8×M] state trajectory (RK4-integrated, smooth)
%   u0   — [4×M] control trajectory
%   t_f0 — scalar release time
% =========================================================================

M  = scp.M;
N  = p.N;
nx = 2*N;

% ==========================================================================
%  STEP 1: Get controls and release time from chosen initialiser
% ==========================================================================
switch lower(method)

    case 'standard'
        fprintf('  Initialising SCP with standard backward-swing guess...\n');
        nlp_tmp.M = M;
        nlp_tmp.N = N;
        z0_col = initialize_collocation_guess(nlp_tmp, p);
        tau0 = reshape(z0_col(2*N*M+1 : 3*N*M), N, M);
        t_f0 = z0_col(end);

    case 'gcs'
        fprintf('  Initialising SCP with GCS-derived guess...\n');
        config_name = p.current_config;
        p_tmp = p;
        p_tmp.opt.M = M;
        [z0_gcs, ~] = gcs_to_dynamics_v2(config_name, 'direct_collocation', p_tmp);
        tau0 = reshape(z0_gcs(2*N*M+1 : 3*N*M), N, M);
        t_f0 = z0_gcs(end);
        fprintf('  GCS guess: t_f=%.2f s, max|tau|=%.1f N·m\n', t_f0, max(abs(tau0(:))));

    otherwise
        error('scp_initialize_trajectory: unknown method "%s"', method);
end

% Clamp controls and release time
u0   = tau0;
for k = 1:M
    u0(:,k) = max(p.lim.tau_min, min(p.lim.tau_max, u0(:,k)));
end
t_f0 = max(scp.t_release_min, min(scp.t_release_max, t_f0));

% ==========================================================================
%  STEP 2: RK4 rollout — stable integration to get a smooth x0
%
%  Forward Euler blows up for robot arm dynamics at h~0.02s.
%  RK4 is stable and gives a smooth reference trajectory.
%
%  NOTE: the QP equality constraints encode Forward Euler linearised
%  around x_bar.  The defect between x_bar (RK4) and the Forward Euler
%  prediction is handled by the beq RHS in scp_build_subproblem:
%    beq = h*(f_k - A_k*x_bar_k - B_k*u_bar_k)
%  which correctly accounts for the linearisation residual.
% ==========================================================================
h   = t_f0 / M;
x0  = zeros(nx, M);
x0(:,1) = [p.q0; p.qdot0];

for k = 1:M-1
    xk = x0(:,k);
    uk = u0(:,k);

    % RK4 stages
    k1 = arm_dynamics_continuous(xk,             uk, p);
    k2 = arm_dynamics_continuous(xk + h/2 * k1, uk, p);
    k3 = arm_dynamics_continuous(xk + h/2 * k2, uk, p);
    k4 = arm_dynamics_continuous(xk + h   * k3, uk, p);

    x0(:, k+1) = xk + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

% ==========================================================================
%  STEP 3: Report
% ==========================================================================
q_range  = [min(x0(1:N,:),   [], 'all'), max(x0(1:N,:),   [], 'all')];
qd_range = [min(x0(N+1:end,:),[],'all'), max(x0(N+1:end,:),[],'all')];

fprintf('  RK4 rollout complete:\n');
fprintf('    M=%d, t_f=%.2f s, h=%.4f s\n', M, t_f0, h);
fprintf('    q    range: [%.2f, %.2f] rad\n',   q_range(1),  q_range(2));
fprintf('    qdot range: [%.2f, %.2f] rad/s\n', qd_range(1), qd_range(2));
fprintf('    tau  range: [%.2f, %.2f] N·m\n',   min(u0(:)),  max(u0(:)));
fprintf('    NaN in x0:  %d\n', nnz(isnan(x0)));

end