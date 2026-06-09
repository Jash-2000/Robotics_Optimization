function collision_constraints = add_collision_constraints_casadi(opti, q, p)
% add_collision_constraints_casadi.m
% =========================================================================
% Adds collision avoidance constraints to CasADi Opti stack.
% 
% NOTE: For direct collocation with CasADi, proper collision constraints
% require symbolic collision detection, which is complex. For now, we add
% simple distance-based constraints using numerical evaluation callbacks.
%
% INPUTS:
%   opti – CasADi Opti object
%   q    – [4×M] joint angle decision variables (symbolic)
%   p    – parameter struct with obstacles
%
% =========================================================================

import casadi.*

M = size(q, 2);
N = size(q, 1);

if isempty(p.obstacles)
    fprintf('    No obstacles - skipping collision constraints\n');
    collision_constraints.n_constraints = 0;
    return;
end

fprintf('    Adding collision constraints...\n');

n_constraints = 0;

% For each time point, add collision constraints
for k = 1:M
    q_k = q(:, k);  % State at time k (symbolic)
    
    % For collision constraints with symbolic variables, we use a simplified approach:
    % We add inequality constraints using numerical callback functions
    
    % Add constraint for each link
    for link_i = 1:N
        % Create a function that evaluates collision distance numerically
        % The function takes numeric input and returns numeric output
        collision_dist_func = @(q_eval) eval_link_collision_numeric(q_eval, link_i, p);
        
        % Wrap as CasADi external function
        casadi_ext = casadi.external('link_coll', collision_dist_func);
        
        % Add constraint: dist >= 0 (no collision)
        opti.subject_to(casadi_ext(q_k) >= 0);
        n_constraints = n_constraints + 1;
    end
    
    % Add constraint for object
    collision_obj_func = @(q_eval) eval_object_collision_numeric(q_eval, p);
    casadi_ext_obj = casadi.external('obj_coll', collision_obj_func);
    
    opti.subject_to(casadi_ext_obj(q_k) >= 0);
    n_constraints = n_constraints + 1;
end

fprintf('      Added %d collision constraints (%d time points × %d checks)\n', ...
    n_constraints, M, N+1);

collision_constraints.n_constraints = n_constraints;

end


%% ═══════════════════════════════════════════════════════════════════════
%% HELPER FUNCTIONS
%% ═══════════════════════════════════════════════════════════════════════

function dist = eval_link_collision_numeric(q, link_idx, p)
% Evaluates collision distance for a link (pure numeric function)
% This function is called numerically by CasADi callbacks

% Get joint positions via forward kinematics
[joint_pos, ~] = forward_kinematics(q, p);

% Link segment endpoints
p1 = joint_pos(:, link_idx);
p2 = joint_pos(:, link_idx + 1);

% Query collision
query.type = 'link';
query.p1 = p1;
query.p2 = p2;

[is_collision, details] = check_collision(query, p.obstacles, p);

% Return minimum signed distance
if isempty(details)
    dist = 1.0;  % No obstacles, large positive distance
else
    dists = [details.dist];
    dist = min(dists);
end

end


function dist = eval_object_collision_numeric(q, p)
% Evaluates collision distance for object at end-effector (pure numeric)

% Get end-effector position
[joint_pos, ~] = forward_kinematics(q, p);
ee_pos = joint_pos(:, end);

% Object orientation
alpha_N = sum(q);

% Object position with offset
R_N = [cos(alpha_N), -sin(alpha_N);
       sin(alpha_N),  cos(alpha_N)];
obj_pos = ee_pos + R_N * p.obj.r_gc;

% Query collision
query.type = 'object';
query.pos = obj_pos;
query.theta = alpha_N;

[is_collision, details] = check_collision(query, p.obstacles, p);

% Return minimum signed distance
if isempty(details)
    dist = 1.0;
else
    dists = [details.dist];
    dist = min(dists);
end

end