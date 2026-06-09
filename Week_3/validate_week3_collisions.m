% validate_week3_collisions.m
% ===========================================================================
% POST-OPTIMIZATION COLLISION VALIDATION
% ===========================================================================
% Checks whether Week 3 optimized trajectories are actually collision-free
% using the original check_collision.m function from Week 1.
% ===========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WEEK 3: POST-OPTIMIZATION COLLISION VALIDATION\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Load results ──────────────────────────────────────────────────────────
fprintf('[1/3] Loading Week 3 results...\n');

addpath('/mnt/project');  % Week 1 functions

load('week3_all_results.mat', 'results', 'p');

configs = {'simple', 'moderate', 'hard'};

fprintf('  ✓ Results loaded\n\n');

%% ── Validate each configuration ───────────────────────────────────────────
fprintf('[2/3] Checking collisions for each configuration...\n\n');

collision_report = struct();

for c_idx = 1:length(configs)
    config_name = configs{c_idx};
    
    fprintf('  ───────────────────────────────────────────────────────────\n');
    fprintf('  Configuration: %s\n', upper(config_name));
    fprintf('  ───────────────────────────────────────────────────────────\n');
    
    % Extract trajectory
    traj = results.(config_name).traj;
    
    % Load obstacles for this config
    obstacles = load_obstacle_config(config_name, p);
    p_temp = p;
    p_temp.obstacles = obstacles;
    
    % Check collisions at each time point
    n_collisions_arm = 0;
    n_collisions_obj = 0;
    collision_times_arm = [];
    collision_times_obj = [];
    
    for k = 1:length(traj.t)
        t_k = traj.t(k);
        q_k = traj.q(:, k);
        
        % Get joint positions
        [joint_pos, ~] = forward_kinematics(q_k, p_temp);
        
        % Check each link
        for link_i = 1:p.N
            p1 = joint_pos(:, link_i);
            p2 = joint_pos(:, link_i + 1);
            
            query.type = 'link';
            query.p1 = p1;
            query.p2 = p2;
            
            [is_collision, details] = check_collision(query, obstacles, p_temp);
            
            if is_collision
                n_collisions_arm = n_collisions_arm + 1;
                collision_times_arm = [collision_times_arm; t_k];
                fprintf('    ⚠ ARM COLLISION at t=%.3f s (link %d)\n', t_k, link_i);
                break;  % Only count once per time point
            end
        end
        
        % Check object at end-effector
        ee_pos = joint_pos(:, end);
        alpha_N = sum(q_k);
        R_N = [cos(alpha_N), -sin(alpha_N);
               sin(alpha_N),  cos(alpha_N)];
        obj_pos = ee_pos + R_N * p.obj.r_gc;
        
        query.type = 'object';
        query.pos = obj_pos;
        query.theta = alpha_N;
        
        [is_collision, details] = check_collision(query, obstacles, p_temp);
        
        if is_collision
            n_collisions_obj = n_collisions_obj + 1;
            collision_times_obj = [collision_times_obj; t_k];
            fprintf('    ⚠ OBJECT COLLISION at t=%.3f s\n', t_k);
        end
    end
    
    % Summary for this config
    total_collisions = n_collisions_arm + n_collisions_obj;
    
    if total_collisions == 0
        fprintf('    ✓ NO COLLISIONS - Trajectory is collision-free!\n');
        status = 'COLLISION-FREE';
    else
        fprintf('    ✗ COLLISIONS DETECTED:\n');
        fprintf('      - Arm collisions: %d time points\n', n_collisions_arm);
        fprintf('      - Object collisions: %d time points\n', n_collisions_obj);
        fprintf('      - Total: %d / %d time points (%.1f%%)\n', ...
            total_collisions, length(traj.t), 100*total_collisions/length(traj.t));
        status = 'HAS COLLISIONS';
    end
    
    % Store report
    collision_report.(config_name).status = status;
    collision_report.(config_name).n_arm = n_collisions_arm;
    collision_report.(config_name).n_obj = n_collisions_obj;
    collision_report.(config_name).n_total = total_collisions;
    collision_report.(config_name).collision_times_arm = collision_times_arm;
    collision_report.(config_name).collision_times_obj = collision_times_obj;
    
    fprintf('\n');
end

%% ── Summary table ─────────────────────────────────────────────────────────
fprintf('[3/3] Validation Summary:\n\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  COLLISION VALIDATION SUMMARY\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

fprintf('%-12s | %15s | %8s | %8s | %8s\n', ...
    'Config', 'Status', 'Arm', 'Object', 'Total');
fprintf('%s\n', repmat('─', 1, 70));

for c_idx = 1:length(configs)
    config_name = configs{c_idx};
    rep = collision_report.(config_name);
    
    fprintf('%-12s | %15s | %8d | %8d | %8d\n', ...
        upper(config_name), rep.status, rep.n_arm, rep.n_obj, rep.n_total);
end

fprintf('%s\n', repmat('─', 1, 70));
fprintf('\n');

%% ── Determine pass/fail ───────────────────────────────────────────────────
all_collision_free = true;
for c_idx = 1:length(configs)
    config_name = configs{c_idx};
    if collision_report.(config_name).n_total > 0
        all_collision_free = false;
        break;
    end
end

if all_collision_free
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('  ✓ ALL CONFIGURATIONS ARE COLLISION-FREE!\n');
    fprintf('  Penalty method successfully avoided all obstacles.\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
else
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('  ⚠ SOME CONFIGURATIONS HAVE COLLISIONS\n');
    fprintf('  Consider:\n');
    fprintf('    1. Increase collision penalty weight (currently 10000)\n');
    fprintf('    2. Increase collocation points M for finer discretization\n');
    fprintf('    3. Implement hard constraints instead of penalty method\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
end

fprintf('\n');

%% ── Save report ───────────────────────────────────────────────────────────
save('week3_collision_report.mat', 'collision_report');
fprintf('Collision report saved: week3_collision_report.mat\n\n');