function [x_new, u_new] = scp_unpack_solution(z_sol, qp)
% scp_unpack_solution.m
% =========================================================================
% Extract state / control trajectories from the QP solution vector.
%
% NOTE: t_f is no longer part of the QP. It is updated separately via
% scp_update_release_time after each SCP iteration.
%
% INPUTS:
%   z_sol — [nz×1] optimal primal from quadprog
%   qp    — struct from scp_build_subproblem
%
% OUTPUTS:
%   x_new — [8×M] new state trajectory
%   u_new — [4×M] new control trajectory
% =========================================================================

M  = qp.M;
nx = qp.nx;
nu = qp.nu;

x_new = zeros(nx, M);
u_new = zeros(nu, M);

for k = 1:M
    x_new(:, k) = z_sol(qp.idx_x(k));
    u_new(:, k) = z_sol(qp.idx_u(k));
end

end