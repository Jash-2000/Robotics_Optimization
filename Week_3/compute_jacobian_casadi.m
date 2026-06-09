function J = compute_jacobian_casadi(q, p)
% compute_jacobian_casadi.m
% =========================================================================
% Computes the end-effector Jacobian using CasADi symbolic variables.
%
% INPUTS:
%   q – [4×1] joint angles (can be casadi.SX or numeric)
%   p – parameter struct
%
% OUTPUT:
%   J – [2×4] Jacobian matrix: v_ee = J * qdot
%       where v_ee = [v_x; v_y] is end-effector velocity in world frame
%
% DERIVATION:
%   End-effector position: r_ee(q) from forward kinematics
%   Jacobian: J = ∂r_ee/∂q
%
%   For planar arm:
%   J_ij = ∂r_ee_i / ∂q_j
%
% =========================================================================

import casadi.*

N = p.N;

% Compute cumulative angles
alpha1 = q(1);
alpha2 = alpha1 + q(2);
alpha3 = alpha2 + q(3);
alpha4 = alpha3 + q(4);

% Initialize Jacobian columns using direct computation
% Column 1: derivative w.r.t. q1 (all links move)
J_col1_x = p.l(1) * cos(alpha1) + p.l(2) * cos(alpha2) + ...
           p.l(3) * cos(alpha3) + p.l(4) * cos(alpha4);
J_col1_y = -p.l(1) * sin(alpha1) - p.l(2) * sin(alpha2) - ...
           p.l(3) * sin(alpha3) - p.l(4) * sin(alpha4);

% Column 2: derivative w.r.t. q2 (links 2,3,4 move)
J_col2_x = p.l(2) * cos(alpha2) + p.l(3) * cos(alpha3) + p.l(4) * cos(alpha4);
J_col2_y = -p.l(2) * sin(alpha2) - p.l(3) * sin(alpha3) - p.l(4) * sin(alpha4);

% Column 3: derivative w.r.t. q3 (links 3,4 move)
J_col3_x = p.l(3) * cos(alpha3) + p.l(4) * cos(alpha4);
J_col3_y = -p.l(3) * sin(alpha3) - p.l(4) * sin(alpha4);

% Column 4: derivative w.r.t. q4 (link 4 moves)
J_col4_x = p.l(4) * cos(alpha4);
J_col4_y = -p.l(4) * sin(alpha4);

% Assemble Jacobian matrix using horzcat (CasADi-compatible)
if isa(q, 'casadi.SX') || isa(q, 'casadi.MX')
    J = horzcat([J_col1_x; J_col1_y], [J_col2_x; J_col2_y], ...
                [J_col3_x; J_col3_y], [J_col4_x; J_col4_y]);
else
    J = [[J_col1_x; J_col1_y], [J_col2_x; J_col2_y], ...
         [J_col3_x; J_col3_y], [J_col4_x; J_col4_y]];
end

end