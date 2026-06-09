function xdot = arm_ode_free(t, x, p)
% arm_ode_free.m
% =========================================================================
% ODE right-hand side for the N-link planar robotic arm AFTER object
% release. The object is no longer attached, so the arm's effective mass
% and inertia are reduced.
%
% PROPER IMPLEMENTATION REQUIRES:
%   Run derive_dynamics_free.m to generate M_free_func.m, C_free_func.m,
%   G_free_func.m (symbolic derivation with object contribution removed).
%
% STUB IMPLEMENTATION (current):
%   Uses the full dynamics (M_func, C_func, G_func) with zero torque.
%   This is APPROXIMATE but functional for Week 1 validation.
%
% INPUTS:
%   t  – current time [s]
%   x  – [2N×1] state vector: [q; qdot]
%   p  – parameter struct from params.m
%
% OUTPUT:
%   xdot – [2N×1] state derivative: [qdot; qddot]
%
% TODO: Replace with proper free-arm dynamics once derive_dynamics_free.m
%       is run to generate M_free_func, C_free_func, G_free_func.
% =========================================================================

% STUB: Call full dynamics with zero torque
% After release, arm is unpowered (coasting / gravity only)
tau_func = @(t, q, qdot) zeros(p.N, 1);   % zero control torque

xdot = arm_ode(t, x, tau_func, p);

% NOTE: This uses the full M(q), C(q,qdot), G(q) which include the object's
% inertial contribution. For accurate post-release dynamics, derive the
% free-arm dynamics matrices symbolically and replace M_func, C_func, G_func
% with M_free_func, C_free_func, G_free_func in the arm_ode call above.

end
