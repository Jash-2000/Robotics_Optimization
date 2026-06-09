function [J_ee_lin, J_ee_ang, J_obj_lin, J_obj_ang] = jacobians(q, p)
% jacobians.m
% =========================================================================
% Computes the geometric Jacobians for a planar N-link arm:
%   1. End-effector (gripper tip) linear and angular Jacobians
%   2. Object CoM linear and angular Jacobians
%
% INPUTS:
%   q  – [N×1] relative joint angles [rad]
%   p  – parameter struct from params.m
%
% OUTPUTS:
%   J_ee_lin  – [2×N] linear velocity Jacobian of end-effector
%   J_ee_ang  – [1×N] angular velocity Jacobian of end-effector
%   J_obj_lin – [2×N] linear velocity Jacobian of object CoM
%   J_obj_ang – [1×N] angular velocity Jacobian of object CoM
%
%   Velocities:
%       v_ee  = J_ee_lin  * qdot       [2×1, m/s]
%       w_ee  = J_ee_ang  * qdot       [scalar, rad/s]
%       v_obj = J_obj_lin * qdot       [2×1, m/s]
%       w_obj = J_obj_ang * qdot       [scalar, rad/s]
%
% METHOD (Geometric Jacobian for planar arm):
%   For a planar revolute joint i, its contribution to the linear
%   velocity of a point p downstream is:
%       z_i × (p - p_joint_i)   where z_i = [0;0;1] (out-of-plane)
%   In 2D this simplifies to:
%       [-(p_y - joint_i_y);  (p_x - joint_i_x)]
%   The angular contribution of joint i to any downstream body = 1.
% =========================================================================

N = p.N;

% Get joint positions and end-effector via FK
[joint_xy, ~, ee_xy] = forward_kinematics(q, p);

% Absolute angle of last link (for object CoM offset rotation)
alpha_N = sum(q);
R_N = [cos(alpha_N), -sin(alpha_N);
       sin(alpha_N),  cos(alpha_N)];

% Object CoM in world frame
p_obj = ee_xy + R_N * p.obj.r_gc;   % [2×1]

%% ── End-effector Jacobians ───────────────────────────────────────────────
J_ee_lin = zeros(2, N);
J_ee_ang = ones(1, N);      % planar: each revolute joint adds 1 to omega

for i = 1:N
    % Vector from joint i to end-effector
    r = ee_xy - joint_xy(:, i);   % [2×1]
    % Cross product z × r in 2D: [-r_y; r_x]
    J_ee_lin(:, i) = [-r(2); r(1)];
end

%% ── Object CoM Jacobians ─────────────────────────────────────────────────
J_obj_lin = zeros(2, N);
J_obj_ang = ones(1, N);     % object angular velocity = sum of all joint rates

for i = 1:N
    r = p_obj - joint_xy(:, i);   % [2×1]
    J_obj_lin(:, i) = [-r(2); r(1)];
end

% Note: J_obj_ang = J_ee_ang = [1,1,...,1] for a planar chain.
% The object CoM offset r_gc rotates with link N, so its contribution
% to angular velocity is identical to the end-effector angular velocity.

end
