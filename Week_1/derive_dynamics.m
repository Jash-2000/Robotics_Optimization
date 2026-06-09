function derive_dynamics(p)
% derive_dynamics.m
% =========================================================================
% Derives the full Lagrangian equations of motion for the N-link planar arm
% symbolically using MATLAB's Symbolic Toolbox, then saves fast numerical
% functions to disk.
%
% The arm carries an object in its gripper (link N). The object's inertial
% contribution is INCLUDED in the dynamics during Phase 1 (pre-release) by
% augmenting the last link's effective mass and inertia.
%
% OUTPUT FILES (saved to current directory):
%   M_func.m   – mass matrix        M(q)         [N×N]
%   C_func.m   – Coriolis matrix    C(q,qdot)    [N×N]
%   G_func.m   – gravity vector     G(q)         [N×1]
%   T_func.m   – kinetic energy     T(q,qdot)    [scalar]
%   V_func.m   – potential energy   V(q)         [scalar]
%
% USAGE:
%   p = params();
%   derive_dynamics(p);   % run once; slow due to symbolic computation
%
% CONVENTIONS:
%   q_i  = absolute angle of link i from horizontal [rad]
%   The relationship q_abs(i) = sum(q(1:i)) where q are the input
%   RELATIVE joint angles. Inside this function we work with ABSOLUTE
%   angles (alpha) for FK clarity, then substitute back.
% =========================================================================

fprintf('\n=== Deriving Lagrangian Dynamics (symbolic) ===\n');
fprintf('    N = %d links. This may take 30–120 s ...\n\n', p.N);
tic;

N  = p.N;
g_val = p.g;

%% ── Symbolic variables ───────────────────────────────────────────────────
% Relative joint angles and velocities
q    = sym('q',    [N,1], 'real');
qdot = sym('qdot', [N,1], 'real');

% Absolute (cumulative) angle of each link from horizontal
alpha = sym(zeros(N,1));
for i = 1:N
    alpha(i) = sum(q(1:i));
end

%% ── Forward kinematics for each link CoM ────────────────────────────────
% Position of joint i (proximal end of link i):
%   p_joint_i = p_joint_{i-1} + l_{i-1}*[cos(alpha_{i-1}); sin(alpha_{i-1})]
% Position of CoM of link i:
%   p_com_i   = p_joint_i + lc_i*[cos(alpha_i); sin(alpha_i)]

joint_pos = sym(zeros(2, N+1));   % columns: joint 0..N positions
joint_pos(:,1) = [0; p.y0];       % base at (0, y0)

for i = 1:N
    joint_pos(:, i+1) = joint_pos(:, i) + ...
        p.l(i) * [cos(alpha(i)); sin(alpha(i))];
end

com_pos = sym(zeros(2, N));
for i = 1:N
    com_pos(:,i) = joint_pos(:,i) + ...
        p.lc(i) * [cos(alpha(i)); sin(alpha(i))];
end

%% ── Augment last link with object inertia (Phase 1: object held) ─────────
% Object CoM in world frame:
%   p_obj = p_ee + R(alpha_N) * r_gc
% where p_ee = joint_pos(:, N+1) = tip of gripper
% For Lagrangian purposes, treat the object as an additional point mass
% at p_obj, plus a rotational inertia I_obj (rotating at alpha_N rate).

m_aug = p.m;        % copy link masses
I_aug = p.I;        % copy link inertias (about CoM)

% Effective last-link parameters WITH object contribution via
% parallel-axis theorem on the combined body is complex; instead we
% add the object as a SEPARATE body (cleaner and exact):
% We'll carry N+1 bodies: N links + 1 object.

% Object CoM position in world frame
R_N    = [cos(alpha(N)), -sin(alpha(N)); ...
          sin(alpha(N)),  cos(alpha(N))];
p_obj  = joint_pos(:, N+1) + R_N * sym(p.obj.r_gc);

% Object velocity (Jacobian × qdot computed during KE)
% Handled below in the kinetic energy sum.

%% ── Kinetic & Potential energy ───────────────────────────────────────────
T = sym(0);
V = sym(0);

for i = 1:N
    % Velocity of CoM of link i
    v_com_i = jacobian(com_pos(:,i), q) * qdot;   % 2×1

    % Kinetic energy: translational + rotational
    T = T + sym(1/2) * m_aug(i) * (v_com_i.' * v_com_i) ...
          + sym(1/2) * I_aug(i) * (sum(qdot(1:i)))^2;

    % Potential energy (y-component of CoM, ground at y=0)
    V = V + m_aug(i) * g_val * com_pos(2,i);
end

% Add object contribution
v_obj = jacobian(p_obj, q) * qdot;   % 2×1 linear velocity of object CoM
omega_obj = sum(qdot);               % angular velocity = sum of joint rates

T = T + sym(1/2)*p.obj.mass*(v_obj.'*v_obj) ...
      + sym(1/2)*p.obj.I*omega_obj^2;
V = V + p.obj.mass * g_val * p_obj(2);

fprintf('  Energies computed. Deriving M, C, G ...\n');

%% ── Mass matrix M(q) via T = (1/2) qdot' M qdot ────────────────────────
% M_ij = d²T / d(qdot_i) d(qdot_j)   (Hessian of T w.r.t. qdot)
M = sym(zeros(N,N));
for i = 1:N
    for j = 1:N
        M(i,j) = diff(diff(T, qdot(i)), qdot(j));
    end
end
M = simplify(M, 'Steps', 20);
fprintf('  M done.\n');

%% ── Gravity vector G(q) = dV/dq ─────────────────────────────────────────
G_vec = sym(zeros(N,1));
for i = 1:N
    G_vec(i) = diff(V, q(i));
end
G_vec = simplify(G_vec, 'Steps', 20);
fprintf('  G done.\n');

%% ── Coriolis / centrifugal matrix C(q, qdot) via Christoffel symbols ────
% C_ij = sum_k  Gamma_{ijk} * qdot_k
% Gamma_{ijk} = (1/2)*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
C = sym(zeros(N,N));
for i = 1:N
    for j = 1:N
        cij = sym(0);
        for k = 1:N
            Gamma_ijk = sym(1/2) * ( diff(M(i,j), q(k)) ...
                                   + diff(M(i,k), q(j)) ...
                                   - diff(M(j,k), q(i)) );
            cij = cij + Gamma_ijk * qdot(k);
        end
        C(i,j) = cij;
    end
end
C = simplify(C, 'Steps', 20);
fprintf('  C done.\n');

%% ── Save numerical functions ─────────────────────────────────────────────
fprintf('  Generating MATLAB function files ...\n');

matlabFunction(M,     'File','M_func',   'Vars',{q},       'Optimize',true);
matlabFunction(C,     'File','C_func',   'Vars',{q,qdot},  'Optimize',true);
matlabFunction(G_vec, 'File','G_func',   'Vars',{q},       'Optimize',true);
matlabFunction(T,     'File','T_func',   'Vars',{q,qdot},  'Optimize',true);
matlabFunction(V,     'File','V_func',   'Vars',{q},       'Optimize',true);

% Also save joint & CoM position functions (useful for FK + animation)
matlabFunction(joint_pos, 'File','joint_positions_func', ...
    'Vars',{q}, 'Optimize',true);
matlabFunction(com_pos,   'File','com_positions_func',   ...
    'Vars',{q}, 'Optimize',true);
matlabFunction(p_obj,     'File','obj_com_position_func',...
    'Vars',{q}, 'Optimize',true);

elapsed = toc;
fprintf('\n=== Symbolic derivation complete (%.1f s) ===\n\n', elapsed);
fprintf('Generated files:\n');
fprintf('  M_func.m, C_func.m, G_func.m\n');
fprintf('  T_func.m, V_func.m\n');
fprintf('  joint_positions_func.m, com_positions_func.m\n');
fprintf('  obj_com_position_func.m\n\n');

end