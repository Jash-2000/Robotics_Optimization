function [J, arm_coll, flight_coll, info] = shooting_objective(z, p)
% shooting_objective.m
% =========================================================================
% Single-shooting objective function for trajectory optimization.
% Integrates the arm ODE from t=0 to t_release, computes landing miss.
%
% NEW: Includes energy minimization and optimizes release time
%
% INPUTS:
%   z – [4M+1×1] decision variables: [τ₁,...,τ₄ for M intervals, t_release]
%   p – parameter struct from params.m (includes p.obstacles, p.opt.M, etc.)
%
% OUTPUTS:
%   J          – scalar objective: w_pos*(x_land-d)² + w_energy*E_total
%   arm_coll   – [n_checks×1] signed distances during manipulation phase
%   flight_coll – [n_flight×1] signed distances during ballistic phase
%   info       – struct with: .x_land, .E_total, .t_release
%
% OBJECTIVE:
%   J = w_pos * (x_land - d)² + w_energy * E_total
%   where E_total = ∫ τ(t)ᵀ·q̇(t) dt (mechanical energy input)
%
% FAILURE MODES:
%   - ODE integration fails → return large penalty J = 1e6
%   - Landing prediction fails (NaN) → return large penalty
% =========================================================================

%% ── Step 1: Unpack decision variables → τ(t) and t_release ───────────────
[tau_func, t_release] = unpack_torques(z, p);

% Validate release time bounds
if t_release < 0.3 || t_release > 2.5
    fprintf('  ⚠ Release time out of bounds: %.2f s\n', t_release);
    J = 1e6;
    arm_coll = [];
    flight_coll = [];
    info.x_land = NaN;
    info.E_total = NaN;
    info.t_release = t_release;
    return;
end

%% ── Step 2: Integrate ODE from t=0 to t_release ──────────────────────────
q0 = p.q0;
qdot0 = p.qdot0;
y0 = [q0; qdot0];   % [8×1] initial state

tspan = [0, t_release];

% ODE options
ode_opts = p.sim.ode_opts;

% Define ODE function
ode_func = @(t, y) arm_ode(t, y, tau_func, p);

try
    % Integrate with ode45
    [t_vec, y_vec] = ode45(ode_func, tspan, y0, ode_opts);
    
    % Extract final state at release
    q_rel = y_vec(end, 1:p.N)';      
    qdot_rel = y_vec(end, p.N+1:end)'; 
    
catch ME
    % ODE integration failed
    fprintf('  ⚠ ODE integration failed: %s\n', ME.message);
    J = 1e6;
    arm_coll = [];
    flight_coll = [];
    info.x_land = NaN;
    info.E_total = NaN;
    info.t_release = t_release;
    return;
end

%% ── Step 3: Compute energy consumption ───────────────────────────────────
% E_total = ∫ τ(t)ᵀ·q̇(t) dt
% Approximate using trapezoidal rule on ODE output

E_total = 0;
for k = 1:length(t_vec)-1
    % Get state at time k
    q_k = y_vec(k, 1:p.N)';
    qdot_k = y_vec(k, p.N+1:end)';
    
    % Get torque at time k
    tau_k = tau_func(t_vec(k), q_k, qdot_k);
    
    % Power = τᵀ·q̇
    power_k = tau_k' * qdot_k;
    
    % Time step
    dt = t_vec(k+1) - t_vec(k);
    
    % Accumulate energy (trapezoidal rule approximation)
    E_total = E_total + abs(power_k) * dt;
end

%% ── Step 4: Compute landing prediction ───────────────────────────────────
rc = release_condition(q_rel, qdot_rel, p);

if isnan(rc.x_land)
    % Landing prediction failed
    fprintf('  ⚠ Landing prediction failed (NaN)\n');
    J = 1e6;
    arm_coll = [];
    flight_coll = [];
    info.x_land = NaN;
    info.E_total = E_total;
    info.t_release = t_release;
    return;
end

%% ── Step 5: Compute multi-objective ──────────────────────────────────────
% Weights (energy term is regularization)
w_pos = p.opt.w_position;     % Weight on accuracy
w_energy = p.opt.w_energy;    % Weight on energy (regularization)
w_direction = 10;              % Weight on throwing direction (soft constraint)

% Position error term
J_pos = (rc.x_land - p.task.d)^2;

% Energy term (normalized to prevent scale issues)
J_energy = E_total / t_release;  % Divide by typical energy scale

% Direction penalty: penalize negative x-velocity at release
v_x_release = rc.obj_vel(1);  % x-component of release velocity
if v_x_release < 0
    J_direction = abs(v_x_release);  % Penalize backwards throwing
else
    J_direction = 0;  % No penalty for forward throwing
end

% Combined objective
J = w_pos * J_pos + w_energy * J_energy + w_direction * J_direction;

%% ── Step 6: Check arm collisions during manipulation phase ───────────────
n_checks = min(50, length(t_vec));  
check_indices = round(linspace(1, length(t_vec), n_checks));

arm_coll = [];  

for i = 1:length(check_indices)
    idx = check_indices(i);
    q_check = y_vec(idx, 1:p.N)';
    
    % Get all link positions
    [joint_pos, ~, ~] = forward_kinematics(q_check, p);
    
    % Check each link
    for link_i = 1:p.N
        p1 = joint_pos(:, link_i);
        p2 = joint_pos(:, link_i + 1);
        
        query.type = 'link';
        query.p1 = p1;
        query.p2 = p2;
        
        [~, details] = check_collision(query, p.obstacles, p);
        
        if ~isempty(details)
            dists = [details.dist];
            arm_coll = [arm_coll; min(dists)];
        end
    end
    
    % Check object (attached to end-effector)
    alpha_N = sum(q_check);  
    ee_xy = joint_pos(:, end);
    R_N = [cos(alpha_N), -sin(alpha_N);
           sin(alpha_N),  cos(alpha_N)];
    obj_pos = ee_xy + R_N * p.obj.r_gc;
    
    query_obj.type = 'object';
    query_obj.pos = obj_pos;
    query_obj.theta = alpha_N;
    
    [~, details_obj] = check_collision(query_obj, p.obstacles, p);
    
    if ~isempty(details_obj)
        dists_obj = [details_obj.dist];
        arm_coll = [arm_coll; min(dists_obj)];
    end
end

%% ── Step 7: Check flight collisions via ballistic_trajectory ─────────────
% Pass empty t_span to auto-compute until landing
traj = ballistic_trajectory(q_rel, qdot_rel, p, []);

% Check collisions along flight path
n_flight = length(traj.t);
flight_coll = [];

if ~isempty(p.obstacles)
    for k = 1:n_flight
        q_flight.type  = 'object';
        q_flight.pos   = traj.pos(:, k);
        q_flight.theta = traj.theta(k);
        
        [~, details_flight] = check_collision(q_flight, p.obstacles, p);
        
        if ~isempty(details_flight)
            dists_flight = [details_flight.dist];
            flight_coll = [flight_coll; min(dists_flight)];
        end
    end
end

%% ── Pack info ─────────────────────────────────────────────────────────────
info.x_land = rc.x_land;
info.E_total = E_total;
info.t_release = t_release;
info.J_pos = J_pos;
info.J_energy = J_energy;
info.J_direction = J_direction;
info.v_x_release = v_x_release;

end