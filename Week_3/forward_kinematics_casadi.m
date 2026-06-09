function [joint_pos, ee_pos] = forward_kinematics_casadi(q, p)
% forward_kinematics_casadi.m
% =========================================================================
% Computes forward kinematics using CasADi symbolic variables.
% Compatible with both numerical arrays and casadi.SX/MX symbolic types.
%
% INPUTS:
%   q – [4×1] joint angles (can be casadi.SX or numeric)
%   p – parameter struct
%
% OUTPUTS:
%   joint_pos – [2×5] joint positions (base + 4 joints) in world frame
%   ee_pos    – [2×1] end-effector position
%
% =========================================================================

import casadi.*

N = p.N;

% Compute cumulative angles using direct operations (avoid indexing assignment)
alpha1 = q(1);
alpha2 = alpha1 + q(2);
alpha3 = alpha2 + q(3);
alpha4 = alpha3 + q(4);

% Base position [x; y]
pos0 = [0; p.y0];

% Link 1
pos1 = pos0 + [p.l(1) * sin(alpha1); p.l(1) * cos(alpha1)];

% Link 2
pos2 = pos1 + [p.l(2) * sin(alpha2); p.l(2) * cos(alpha2)];

% Link 3
pos3 = pos2 + [p.l(3) * sin(alpha3); p.l(3) * cos(alpha3)];

% Link 4
pos4 = pos3 + [p.l(4) * sin(alpha4); p.l(4) * cos(alpha4)];

% Concatenate positions into matrix using CasADi-compatible function
if isa(q, 'casadi.SX') || isa(q, 'casadi.MX')
    joint_pos = horzcat(pos0, pos1, pos2, pos3, pos4);
else
    joint_pos = [pos0, pos1, pos2, pos3, pos4];
end

% End-effector is the last joint
ee_pos = pos4;

end