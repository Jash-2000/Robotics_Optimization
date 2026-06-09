function [tau_func, t_release] = unpack_torques(z, p)
% unpack_torques.m
% =========================================================================
% Converts decision variable vector z into a torque function handle τ(t).
% Uses PIECEWISE-CONSTANT torque parameterization.
%
% INPUTS:
%   z – [4M+1×1] decision variables: [τ₁(t₁), τ₂(t₁), τ₃(t₁), τ₄(t₁), 
%                                      τ₁(t₂), τ₂(t₂), ..., τ₄(tₘ), t_release]
%   p – parameter struct from params.m (contains p.opt.M)
%
% OUTPUTS:
%   tau_func  – function handle: τ(t,q,qdot) returns [4×1] torque vector at time t
%   t_release – scalar release time [s] extracted from z
%
% PARAMETERIZATION:
%   - First 4M elements: piecewise-constant torques
%   - Last element z(4M+1): release time t_release
%   - Divide [0, t_release] into M intervals: Δt = t_release / M
%   - Interval k spans: [(k-1)*Δt, k*Δt)
%   - τ(t) is constant within each interval
%
% USAGE:
%   [tau_func, t_rel] = unpack_torques(z, p);
%   tau_vec = tau_func(0.5, q, qdot);  % returns [4×1] torque at t=0.5 s
% =========================================================================

M = p.opt.M;
N = p.N;

% Extract release time from last element of z
t_release = z(end);

% Time interval length
dt = t_release / M;

% Extract release time from last element of z
t_release = z(end);

% Time interval length
dt = t_release / M;

% Reshape first 4M elements into [N × M] matrix: z_mat(i,k) = τᵢ at interval k
z_mat = reshape(z(1:4*M), [N, M]);

% Create function handle
% Note: tau_func(t, q, qdot) signature required by arm_ode.m
% We ignore q and qdot (piecewise-constant doesn't depend on state)
tau_func = @(t, q, qdot) torque_at_t(t, z_mat, M, dt);

end

%% ── Nested helper function ───────────────────────────────────────────────
function tau = torque_at_t(t, z_mat, M, dt)
    % Returns [4×1] torque at time t
    % Signature: tau_func(t, q, qdot) for compatibility with arm_ode.m
    % We only use t; q and qdot are ignored for piecewise-constant control
    
    if t < 0
        k = 1;
    elseif t >= M*dt
        k = M;
    else
        % Find which interval t falls into: k = ceil((t+eps)/dt)
        k = min(M, max(1, ceil((t + 1e-12) / dt)));
    end
    
    tau = z_mat(:, k);
end