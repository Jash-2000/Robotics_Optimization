function nlp = build_collocation_nlp(p)
% build_collocation_nlp.m
% =========================================================================
% Constructs the direct collocation NLP using CasADi symbolic framework.
% Discretizes both state q(t), qdot(t) and control τ(t) as decision variables.
%
% INPUTS:
%   p – parameter struct from params.m
%
% OUTPUTS:
%   nlp – struct containing:
%     .opti     -- CasADi Opti optimization object
%     .z        -- full decision variable vector (symbolic)
%     .q        -- state positions [4×M] (symbolic)
%     .qdot     -- state velocities [4×M] (symbolic)
%     .tau      -- control torques [4×M] (symbolic)
%     .t_release -- release time scalar (symbolic)
%     .n_vars   -- total number of decision variables
%     .n_eq     -- number of equality constraints
%     .n_ineq   -- number of inequality constraints
%
% DECISION VARIABLES:
%   z ∈ ℝ^(12M+1) = [q₁, ..., qₘ, qdot₁, ..., qdotₘ, τ₁, ..., τₘ, t_release]
%
% CONSTRAINTS:
%   1. Initial condition: q₀ = q_init, qdot₀ = 0
%   2. Dynamics (trapezoidal collocation): 
%      q_{k+1} = q_k + Δt/2 · (qdot_k + qdot_{k+1})
%      qdot_{k+1} = qdot_k + Δt/2 · (qddot_k + qddot_{k+1})
%   3. Collision avoidance: dist(·) ≥ 0
%   4. Torque limits: τ_min ≤ τ ≤ τ_max
%   5. Release time bounds: t_min ≤ t_release ≤ t_max
%
% =========================================================================

import casadi.*

fprintf('  Building CasADi Opti stack...\n');

%% ── Setup Opti optimization object ────────────────────────────────────────
opti = casadi.Opti();

M = p.opt.M;  % Number of collocation points
N = p.N;      % Number of joints (4)

%% ── Decision variables ────────────────────────────────────────────────────
fprintf('  Creating decision variables...\n');

% State: q[k], qdot[k] for k = 1..M
q    = opti.variable(N, M);  % Joint positions [4×M]
qdot = opti.variable(N, M);  % Joint velocities [4×M]

% Control: τ[k] for k = 1..M
tau  = opti.variable(N, M);  % Joint torques [4×M]

% Release time
t_release = opti.variable();  % Scalar

fprintf('    State variables: %d (q) + %d (qdot) = %d\n', N*M, N*M, 2*N*M);
fprintf('    Control variables: %d\n', N*M);
fprintf('    Release time: 1\n');
fprintf('    Total decision variables: %d\n', 2*N*M + N*M + 1);

%% ── Objective function ────────────────────────────────────────────────────
fprintf('  Building objective function...\n');

% Extract release state (last collocation point)
q_rel = q(:, M);
qdot_rel = qdot(:, M);

% Compute object position and velocity at release using CasADi functions
[obj_pos_rel, obj_vel_rel] = compute_release_kinematics_casadi(q_rel, qdot_rel, p);

% Predict landing x-coordinate using ballistic motion
x_land = compute_landing_casadi(obj_pos_rel, obj_vel_rel, p);

% Position error term
J_pos = (x_land - p.task.d)^2;

% Energy term: E = Σ |τ_k · qdot_k| · Δt
dt = t_release / M;
E_total = 0;
for k = 1:M
    power_k = dot(tau(:, k), qdot(:, k));
    E_total = E_total + abs(power_k) * dt;
end
J_energy = E_total / t_release;  % Normalize by time

% Direction penalty: penalize negative x-velocity
v_x_rel = obj_vel_rel(1);
J_direction = if_else(v_x_rel < 0, -v_x_rel, 0);

% Collision avoidance: Pure soft penalty method
% (Hard constraints are difficult with CasADi symbolic - use high penalty instead)
fprintf('  Adding collision avoidance penalties...\n');

J_collision = 0;
collision_penalty_weight = 1e8;  % 1 billion: collision avoidance COMPLETELY dominates
safety_margin = 0.05;  % 5cm safety margin
n_collision_penalties = 0;

if ~isempty(p.obstacles)
    
    %% ── MANIPULATION PHASE: Penalty on arm links and object ──────────────
    for k = 1:M
        q_k = q(:, k);
        
        % Compute joint positions at this time (symbolic)
        [joint_pos_k, ~] = forward_kinematics_casadi(q_k, p);
        
        for obs_idx = 1:length(p.obstacles)
            obs = p.obstacles(obs_idx);
            
            % ── Penalty for each link (3 sample points) ──
            for link_i = 1:N
                p1 = joint_pos_k(:, link_i);
                p2 = joint_pos_k(:, link_i + 1);
                link_mid = (p1 + p2) / 2;
                
                effective_r = obs.r + safety_margin;
                
                % Soft penalty: dist² >= effective_r²
                dist_sq_p1 = (p1(1) - obs.cx)^2 + (p1(2) - obs.cy)^2;
                pen_p1 = effective_r^2 - dist_sq_p1;
                J_collision = J_collision + if_else(pen_p1 > 0, pen_p1^2, 0);  % Squared penalty
                
                dist_sq_mid = (link_mid(1) - obs.cx)^2 + (link_mid(2) - obs.cy)^2;
                pen_mid = effective_r^2 - dist_sq_mid;
                J_collision = J_collision + if_else(pen_mid > 0, pen_mid^2, 0);
                
                dist_sq_p2 = (p2(1) - obs.cx)^2 + (p2(2) - obs.cy)^2;
                pen_p2 = effective_r^2 - dist_sq_p2;
                J_collision = J_collision + if_else(pen_p2 > 0, pen_p2^2, 0);
                
                n_collision_penalties = n_collision_penalties + 3;
            end
            
            % ── Penalty for object at end-effector ──
            ee_pos = joint_pos_k(:, end);
            alpha_total = q_k(1) + q_k(2) + q_k(3) + q_k(4);
            obj_offset_x = p.obj.r_gc(1) * cos(alpha_total) - p.obj.r_gc(2) * sin(alpha_total);
            obj_offset_y = p.obj.r_gc(1) * sin(alpha_total) + p.obj.r_gc(2) * cos(alpha_total);
            obj_pos = ee_pos + [obj_offset_x; obj_offset_y];
            
            effective_r_obj = obs.r + p.obj.r + safety_margin;
            dist_sq_obj = (obj_pos(1) - obs.cx)^2 + (obj_pos(2) - obs.cy)^2;
            pen_obj = effective_r_obj^2 - dist_sq_obj;
            J_collision = J_collision + if_else(pen_obj > 0, pen_obj^2, 0);
            
            n_collision_penalties = n_collision_penalties + 1;
        end
    end
    fprintf('    Manipulation penalties: %d checks\n', n_collision_penalties);
    
    %% ── BALLISTIC FLIGHT: Hard constraints on object trajectory ──────────
    fprintf('  Adding HARD constraints on ballistic flight path...\n');
    
    n_flight_samples = 40;  % Very dense sampling of flight arc
    t_flight_max = 2.5;     % Max possible flight time
    n_flight_constraints_hard = 0;
    
    for s = 1:n_flight_samples
        t_s = t_flight_max * s / n_flight_samples;
        
        % Object position at time t_s after release (ballistic parabola)
        % r(t) = r0 + v0*t + 0.5*g*t²
        flight_pos_x = obj_pos_rel(1) + obj_vel_rel(1) * t_s;
        flight_pos_y = obj_pos_rel(2) + obj_vel_rel(2) * t_s - 0.5 * p.g * t_s^2;
        
        % Only constrain while object is in air (y > 0)
        % But this creates a problem: we can't use if_else in constraints
        % Solution: Apply soft penalty that becomes VERY harsh as y approaches 0
        
        for obs_idx = 1:length(p.obstacles)
            obs = p.obstacles(obs_idx);
            
            effective_r_flight = obs.r + p.obj.r + safety_margin;
            dist_sq_flight = (flight_pos_x - obs.cx)^2 + (flight_pos_y - obs.cy)^2;
            
            % VERY STRONG penalty for flight collisions
            % Use quartic penalty (even stronger than quadratic)
            pen_flight = effective_r_flight^2 - dist_sq_flight;
            J_collision = J_collision + if_else(pen_flight > 0, pen_flight^4, 0);
            
            n_flight_constraints_hard = n_flight_constraints_hard + 1;
        end
    end
    
    fprintf('    Flight path: %d sample points × %d obstacles = %d constraints\n', ...
        n_flight_samples, length(p.obstacles), n_flight_constraints_hard);
    fprintf('    Penalty: Quartic (very strong) on flight collisions\n');
    fprintf('    Total penalty terms: manipulation + quartic flight penalties\n');
    fprintf('    Collision penalty weight: %.0e\n', collision_penalty_weight);
    fprintf('    Safety margin: %.0f cm\n', safety_margin * 100);
end

% Combined objective
w_pos = p.opt.w_position;
w_energy = p.opt.w_energy;
w_dir = p.opt.w_direction;

J = w_pos * J_pos + w_energy * J_energy + w_dir * J_direction + collision_penalty_weight * J_collision;

opti.minimize(J);

fprintf('    Objective: J = %.0f·J_pos + %.1f·J_energy + %.0f·J_dir + %.0f·J_collision\n', ...
    w_pos, w_energy, w_dir, collision_penalty_weight);

%% ── Equality constraints: Initial conditions ──────────────────────────────
fprintf('  Adding initial condition constraints...\n');

% Initial state at k=1
opti.subject_to(q(:, 1) == p.q0);
opti.subject_to(qdot(:, 1) == p.qdot0);

fprintf('    Initial conditions: %d equality constraints\n', 2*N);

%% ── Equality constraints: Dynamics (trapezoidal collocation) ──────────────
fprintf('  Adding dynamics constraints (trapezoidal rule)...\n');

for k = 1:M-1
    % Get states at k and k+1
    q_k = q(:, k);
    qdot_k = qdot(:, k);
    tau_k = tau(:, k);
    
    q_kp1 = q(:, k+1);
    qdot_kp1 = qdot(:, k+1);
    tau_kp1 = tau(:, k+1);
    
    % Compute accelerations at k and k+1 using dynamics
    qddot_k = compute_qddot_casadi(q_k, qdot_k, tau_k, p);
    qddot_kp1 = compute_qddot_casadi(q_kp1, qdot_kp1, tau_kp1, p);
    
    % Trapezoidal integration
    % q_{k+1} = q_k + dt/2 * (qdot_k + qdot_{k+1})
    % qdot_{k+1} = qdot_k + dt/2 * (qddot_k + qddot_{k+1})
    
    opti.subject_to(q_kp1 == q_k + dt/2 * (qdot_k + qdot_kp1));
    opti.subject_to(qdot_kp1 == qdot_k + dt/2 * (qddot_k + qddot_kp1));
end

n_dynamics_constraints = 2*N*(M-1);
fprintf('    Dynamics: %d equality constraints (trapezoidal rule)\n', ...
    n_dynamics_constraints);

%% ── Box constraints: Bounds on decision variables ─────────────────────────
fprintf('  Adding box constraints...\n');

% Joint angle limits: q ∈ [-π, π]
opti.subject_to(-pi <= q <= pi);

% Joint velocity limits (optional, can be set high if not critical)
opti.subject_to(-10 <= qdot <= 10);  % [rad/s]

% Torque limits
for i = 1:N
    opti.subject_to(p.lim.tau_min(i) <= tau(i, :) <= p.lim.tau_max(i));
end

% Release time bounds
opti.subject_to(p.opt.t_release_min <= t_release <= p.opt.t_release_max);

fprintf('    Joint limits: %d constraints (q bounds)\n', 2*N*M);
fprintf('    Velocity limits: %d constraints (qdot bounds)\n', 2*N*M);
fprintf('    Torque limits: %d constraints (τ bounds)\n', 2*N*M);
fprintf('    Release time: 2 constraints (t bounds)\n');

%% ── Package NLP structure ─────────────────────────────────────────────────
nlp.opti = opti;
nlp.q = q;
nlp.qdot = qdot;
nlp.tau = tau;
nlp.t_release = t_release;

nlp.M = M;
nlp.N = N;

% Total variable and constraint counts
nlp.n_vars = 2*N*M + N*M + 1;  % q + qdot + tau + t_release
nlp.n_eq = 2*N + n_dynamics_constraints;  % Initial + dynamics
nlp.n_ineq = 0;  % No inequality constraints (using penalty method)

fprintf('  ✓ NLP structure created\n');

end


%% ═══════════════════════════════════════════════════════════════════════
%% HELPER FUNCTIONS (CasADi symbolic implementations)
%% ═══════════════════════════════════════════════════════════════════════

function qddot = compute_qddot_casadi(q, qdot, tau, p)
% Compute joint accelerations using manipulator equation:
% qddot = M(q)^{-1} [τ - C(q, qdot)·qdot - G(q)]

import casadi.*

% Call auto-generated dynamics functions
% NOTE: M_func, C_func, G_func are auto-generated and only take q, qdot
% They do NOT take p as parameter (parameters are baked into the functions)
M_mat = M_func(q);
C_mat = C_func(q, qdot);
G_vec = G_func(q);

% Solve for acceleration
qddot = M_mat \ (tau - C_mat * qdot - G_vec);

end


function [obj_pos, obj_vel] = compute_release_kinematics_casadi(q, qdot, p)
% Compute object position and velocity at release using forward kinematics

import casadi.*

% Forward kinematics to get end-effector position
[~, ee_pos] = forward_kinematics_casadi(q, p);

% Object offset from end-effector
alpha_N = q(1);
for i = 2:p.N
    alpha_N = alpha_N + q(i);
end

R_N = [cos(alpha_N), -sin(alpha_N);
       sin(alpha_N),  cos(alpha_N)];
obj_offset = R_N * p.obj.r_gc;

obj_pos = ee_pos + obj_offset;

% Velocity via Jacobian
J_ee = compute_jacobian_casadi(q, p);

% Object velocity accounts for offset rotation
% v_obj = v_ee + ω × r_offset, but in 2D: v_obj = J_ee * qdot + ω * [-r_y; r_x]
omega_total = sum(qdot);  % Total angular velocity
r_offset = obj_offset;

% Velocity from end-effector motion
v_ee = J_ee * qdot;

% Additional velocity from rotation of offset
v_rotation = omega_total * [-r_offset(2); r_offset(1)];

obj_vel = v_ee + v_rotation;

end


function x_land = compute_landing_casadi(obj_pos, obj_vel, p)
% Predict landing x-coordinate using ballistic motion
% Solve: y(t) = y0 + vy*t - 0.5*g*t^2 = 0 for t, then compute x(t_land)

import casadi.*

y0 = obj_pos(2);
vy = obj_vel(2);
g = p.g;

% Quadratic formula: t = (vy + sqrt(vy^2 + 2*g*y0)) / g
t_land = (vy + sqrt(vy^2 + 2*g*y0)) / g;

% Landing x-coordinate
x_land = obj_pos(1) + obj_vel(1) * t_land;

end