function xdot = arm_dynamics_continuous(x, u, p)
% arm_dynamics_continuous.m
% =========================================================================
% Wrapper around arm_ode.m for use in SCP without time dependence.
%
% Converts the constant torque control input u into a function handle
% and calls arm_ode(t, x, tau_func, p).
%
% INPUTS:
%   x   – [8×1] state vector: [q; qdot]
%   u   – [4×1] control torque vector [N·m]
%   p   – parameter struct from params.m
%
% OUTPUT:
%   xdot – [8×1] state derivative: [qdot; qddot]
%
% NOTE: Time parameter t is not used (assumed constant control over
%       a small time step in the SCP discretization).
% =========================================================================

% Create a constant torque function from the input vector u
% arm_ode expects: tau_func(t, q, qdot) → [N×1] torques
% Here we ignore t and just return the constant u
tau_func = @(t, q, qdot) u;

% Call the proper arm_ode with t=0 (arbitrary, since tau doesn't depend on t)
xdot = arm_ode(0, x, tau_func, p);

end