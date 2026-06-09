function [joint_xy, com_xy, ee_xy] = forward_kinematics(q, p)
% forward_kinematics.m
% =========================================================================
% Computes forward kinematics for the N-link planar arm numerically.
%
% INPUTS:
%   q   – [N×1] joint angle vector (RELATIVE angles) [rad]
%   p   – parameter struct from params.m
%
% OUTPUTS:
%   joint_xy  – [2×(N+1)] world-frame positions of all joints
%                column 1 = base, column N+1 = end-effector (gripper tip)
%   com_xy    – [2×N]     world-frame positions of each link CoM
%   ee_xy     – [2×1]     end-effector (gripper tip) position
%
% CONVENTION:
%   Absolute angle of link i from horizontal:
%       alpha(i) = q(1) + q(2) + ... + q(i)
%   Base is at (0, y0).
% =========================================================================

N = p.N;

% Cumulative (absolute) angles
alpha = cumsum(q);          % [N×1]

% Joint positions (base = joint 0, gripper tip = joint N)
joint_xy = zeros(2, N+1);
joint_xy(:,1) = [0; p.y0];     % base

for i = 1:N
    joint_xy(:, i+1) = joint_xy(:, i) + ...
        p.l(i) * [cos(alpha(i)); sin(alpha(i))];
end

% Link CoM positions
com_xy = zeros(2, N);
for i = 1:N
    com_xy(:,i) = joint_xy(:,i) + ...
        p.lc(i) * [cos(alpha(i)); sin(alpha(i))];
end

% End-effector = tip of last link
ee_xy = joint_xy(:, N+1);

end
