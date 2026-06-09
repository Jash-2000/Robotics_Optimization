% main_week1.m
% =========================================================================
% WEEK 1 MASTER SCRIPT
% Comparative Study of Trajectory Optimization for Robotic Arm Throwing
%
% This script runs the complete Week 1 pipeline:
%   1. Load parameters
%   2. Derive symbolic Lagrangian dynamics (if not already done)
%   3. Validate energy conservation (zero-torque test)
%   4. FIXED MODE: Run simulation on all 3 obstacle configs, compare results
%      INTERACTIVE MODE: User places obstacles, single simulation
%   5. Animate simulation
%
% OBSTACLE MODE:
%   Set OBSTACLE_MODE below to:
%   - 'fixed' (default): Loop over 3 predefined configs (simple/moderate/hard)
%   - 'interactive': User clicks to place obstacles (exploratory only)
%
% NOTE: Run derive_dynamics.m ONCE before running this script.
%       The generated M_func.m, C_func.m, G_func.m files must be on path.
%
% MATLAB requirements:
%   - Symbolic Math Toolbox (for derive_dynamics.m, run once)
%   - No other toolboxes required for simulation and animation
% =========================================================================

clc; clear; close all;
addpath(fileparts(mfilename('fullpath')));   % ensure all helpers are on path

%% ═══════════════════════════════════════════════════════════════════════
%%  USER CONFIGURATION
%% ═══════════════════════════════════════════════════════════════════════

% Obstacle mode: 'fixed' (3 predefined configs) or 'interactive' (user input)
OBSTACLE_MODE = 'fixed';   % <<< CHANGE THIS TO 'interactive' FOR EXPLORATORY USE

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   4-Link Planar Robotic Arm — Week 1 Simulation          ║\n');
fprintf('║   Lagrangian Dynamics + Ballistic Throwing               ║\n');
fprintf('║   Mode: %-48s ║\n', upper(OBSTACLE_MODE));
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% ── Step 1: Load parameters ──────────────────────────────────────────────
fprintf('[1/6] Loading parameters ...\n');
p = params();
fprintf('      Robot: %d links, lengths = [%s] m\n', p.N, ...
    num2str(p.l', '%.2f '));
fprintf('      Base height y0 = %.2f m\n', p.y0);
fprintf('      Target at x = %.2f m (ground)\n', p.task.d);
fprintf('      Object shape: %s, mass: %.2f kg\n\n', p.obj.shape, p.obj.mass);

%% ── Step 2: Derive / load dynamics ──────────────────────────────────────
fprintf('[2/6] Checking dynamics functions ...\n');
dynamics_files = {'M_func.m','C_func.m','G_func.m', ...
                  'joint_positions_func.m','obj_com_position_func.m'};
all_exist = all(cellfun(@(f) exist(f,'file')==2, dynamics_files));

if all_exist
    fprintf('      Found pre-generated dynamics files. Skipping derivation.\n\n');
else
    fprintf('      Dynamics not found. Running symbolic derivation ...\n');
    fprintf('      (This takes 30–120 s. Run once only.)\n\n');
    derive_dynamics(p);
end

%% ── Step 3: Validate energy conservation ─────────────────────────────────
fprintf('[3/6] Validating energy conservation ...\n');
validate_energy_conservation(p);

%% ═══════════════════════════════════════════════════════════════════════
%%  FIXED MODE: Loop over 3 obstacle configurations
%% ═══════════════════════════════════════════════════════════════════════

if strcmpi(OBSTACLE_MODE, 'fixed')
    
    fprintf('[4/6] Visualizing fixed obstacle configurations ...\n');
    plot_obstacle_configs(p);
    fprintf('      Figure created: 3-config overview\n\n');
    
    config_names = {'simple', 'moderate', 'hard'};
    n_configs = 3;
    
    % Storage for results
    results = struct('config', {}, 'obstacles', {}, 'arm_sol', {}, 'traj', {}, ...
                     'arm_colls', {}, 'flight_colls', {}, 'x_land', {}, 'miss_cm', {});
    
    fprintf('[5/6] Running simulation on all %d configurations ...\n', n_configs);
    
    for c = 1:n_configs
        config_name = config_names{c};
        fprintf('\n  ┌─ Config %d/%d: %s %s\n', c, n_configs, upper(config_name), ...
            repmat('─', 1, 40 - length(config_name)));
        
        % Load obstacles
        obstacles = load_obstacle_config(config_name, p);
        fprintf('  │ Obstacles loaded: %d\n', numel(obstacles));
        
        % Run forward simulation
        fprintf('  │ Running ODE integration ...\n');
        [arm_sol, traj, arm_colls, flight_colls] = run_simulation(obstacles, p);
        
        % Print results
        fprintf('  │ Release: (%.3f, %.3f) m at %.3f m/s, angle %.1f°\n', ...
            traj.r0(1), traj.r0(2), norm(traj.v0), atan2d(traj.v0(2), traj.v0(1)));
        
        if traj.landed
            miss_cm = abs(traj.x_land - p.task.d) * 100;
            fprintf('  │ Landing: x = %.3f m (target: %.3f m)\n', ...
                traj.x_land, p.task.d);
            fprintf('  │ Miss distance: %.2f cm\n', miss_cm);
        else
            miss_cm = NaN;
            fprintf('  │ Landing: FAILED (never hit ground)\n');
        end
        
        fprintf('  │ Arm collisions: %s\n', bool2str(any(arm_colls)));
        fprintf('  │ Flight collisions: %s\n', bool2str(any(flight_colls)));
        fprintf('  └%s\n', repmat('─', 1, 60));
        
        % Store results
        results(c).config = config_name;
        results(c).obstacles = obstacles;
        results(c).arm_sol = arm_sol;
        results(c).traj = traj;
        results(c).arm_colls = arm_colls;
        results(c).flight_colls = flight_colls;
        results(c).x_land = traj.x_land;
        results(c).miss_cm = miss_cm;
        
        % Animate and plot this config immediately
        fprintf('\n  ┌─ Animating Config %d/%d: %s %s\n', c, n_configs, ...
            upper(config_name), repmat('─', 1, 35 - length(config_name)));
        fprintf('  │ Starting animation + plots for %s config ...\n', upper(config_name));
        fprintf('  │ Close animation window to proceed to next config.\n');
        fprintf('  └%s\n', repmat('─', 1, 60));
        
        animate_simulation(arm_sol, traj, obstacles, p);
        
        fprintf('\n  Animation and plots complete for Config %d/%d: %s\n\n', ...
            c, n_configs, upper(config_name));
    end
    
    %% ── Print comparison summary table ───────────────────────────────────
    fprintf('\n[6/6] ═══════════════ COMPARISON SUMMARY ═══════════════\n\n');
    fprintf('  %-12s | %-15s | %-15s | %-12s | %-10s\n', ...
        'Config', 'Arm Collision', 'Flight Collision', 'Landing (m)', 'Miss (cm)');
    fprintf('  %s\n', repmat('─', 1, 78));
    
    for c = 1:n_configs
        arm_str = bool2str(any(results(c).arm_colls));
        flight_str = bool2str(any(results(c).flight_colls));
        
        if isnan(results(c).miss_cm)
            land_str = '   ---';
            miss_str = '   ---';
        else
            land_str = sprintf('%7.3f', results(c).x_land);
            miss_str = sprintf('%7.2f', results(c).miss_cm);
        end
        
        fprintf('  %-12s | %-15s | %-15s | %-12s | %-10s\n', ...
            upper(results(c).config), arm_str, flight_str, land_str, miss_str);
    end
    
    fprintf('  %s\n', repmat('─', 1, 78));
    fprintf('  Target distance: %.2f m\n', p.task.d);
    fprintf('  Note: Week 1 uses FIXED SINUSOIDAL TORQUES (not optimized)\n');
    fprintf('═════════════════════════════════════════════════════════\n\n');
    
    fprintf('All 3 configurations simulated, animated, and plotted.\n');
    fprintf('Total figures generated: %d (4 per config × 3 configs)\n\n', n_configs * 4);

%% ═══════════════════════════════════════════════════════════════════════
%%  INTERACTIVE MODE: User places obstacles
%% ═══════════════════════════════════════════════════════════════════════

elseif strcmpi(OBSTACLE_MODE, 'interactive')
    
    fprintf('[4/6] Drawing environment ...\n');
    ax_env = draw_environment(p);
    fprintf('      Environment drawn.\n\n');
    
    fprintf('[5/6] Interactive obstacle placement ...\n');
    obstacles = place_obstacles_interactive(p, ax_env);
    fprintf('      Obstacles placed: %d\n\n', numel(obstacles));
    
    fprintf('[6/6] Running forward simulation ...\n');
    [arm_sol, traj, arm_colls, flight_colls] = run_simulation(obstacles, p);
    
    % Print single-run summary
    fprintf('\n═══════════════ SIMULATION SUMMARY ═══════════════\n');
    fprintf('  Obstacles placed    : %d\n', numel(obstacles));
    fprintf('  Release position    : (%.4f, %.4f) m\n', traj.r0(1), traj.r0(2));
    fprintf('  Release speed       : %.4f m/s\n', norm(traj.v0));
    fprintf('  Release angle       : %.2f deg\n', atan2d(traj.v0(2),traj.v0(1)));
    if traj.landed
        fprintf('  Landing x           : %.4f m\n', traj.x_land);
        fprintf('  Target x            : %.4f m\n', p.task.d);
        fprintf('  Miss distance       : %.4f m  (%.2f cm)\n', ...
            abs(traj.x_land - p.task.d), abs(traj.x_land - p.task.d)*100);
    else
        fprintf('  Landing             : FAILED (never hit ground)\n');
    end
    fprintf('  Arm collisions      : %s\n', bool2str(any(arm_colls)));
    fprintf('  Flight collisions   : %s\n', bool2str(any(flight_colls)));
    fprintf('═══════════════════════════════════════════════════\n\n');
    
    fprintf('Starting animation ...\n');
    close(get(ax_env,'Parent'));   % close static environment figure
    animate_simulation(arm_sol, traj, obstacles, p);
    
else
    error('Invalid OBSTACLE_MODE: "%s". Must be ''fixed'' or ''interactive''.', ...
        OBSTACLE_MODE);
end

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   Week 1 Complete                                         ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%%  Local helper: run a single simulation
%% ═══════════════════════════════════════════════════════════════════════
function [arm_sol, traj, arm_colls, flight_colls] = run_simulation(obstacles, p)
% Runs forward simulation with fixed sinusoidal torque, returns results

% Simple open-loop torque controller for Week 1 (NOT OPTIMIZED)
% tau_i(t) = A_i * sin(omega_i * t + phi_i)
tau_amp   = [1.5;  8;  0.4;  0.5];      % [N·m]   amplitude per joint
tau_omega = [1; 0.30; 0.35; 0.4];   % [rad/s] frequency per joint
tau_phi   = [0; -0.3; -0.6; 0];       % [rad]   phase offset per joint

tau_func = @(t, q, qdot) compute_torque(t, q, qdot, p, ...
    tau_amp, tau_omega, tau_phi);

% ODE integration
x0      = [p.q0; p.qdot0];
t_span  = [0, p.sim.t_end];
t_eval  = (0 : p.sim.dt : p.sim.t_end)';

[t_sol, x_sol] = ode45( ...
    @(t,x) arm_ode(t, x, tau_func, p), ...
    t_eval, x0, p.sim.ode_opts);

arm_sol.t = t_sol;
arm_sol.y = x_sol';   % [2N × K]

% Find release state
i_rel   = find(t_sol >= p.sim.t_release, 1);
if isempty(i_rel)
    error('Release time %.2f s exceeds simulation end %.2f s.', ...
        p.sim.t_release, p.sim.t_end);
end
q_rel   = x_sol(i_rel, 1:p.N)';
qd_rel  = x_sol(i_rel, p.N+1:2*p.N)';

% Compute ballistic trajectory
traj = ballistic_trajectory(q_rel, qd_rel, p, []);

% Check arm workspace collisions
n_arm_steps = length(t_sol);
arm_colls = false(1, n_arm_steps);
if ~isempty(obstacles)
    for k = 1:n_arm_steps
        q_k = x_sol(k, 1:p.N)';
        [jxy_k, ~, ~] = forward_kinematics(q_k, p);
        for i = 1:p.N
            ql.type = 'link';
            ql.p1   = jxy_k(:,i);
            ql.p2   = jxy_k(:,i+1);
            [col_i,~] = check_collision(ql, obstacles, p);
            if col_i
                arm_colls(k) = true;
                break;
            end
        end
    end
end

% Check flight path collisions
n_flight = length(traj.t);
flight_colls = false(1, n_flight);
if ~isempty(obstacles)
    for k = 1:n_flight
        q_check.type  = 'object';
        q_check.pos   = traj.pos(:,k);
        q_check.theta = traj.theta(k);
        [flight_colls(k), ~] = check_collision(q_check, obstacles, p);
    end
end

end


%% ═══════════════════════════════════════════════════════════════════════
%%  Local helper: simple torque controller for Week 1 testing
%% ═══════════════════════════════════════════════════════════════════════
function tau = compute_torque(t, q, qdot, p, amp, omega_t, phi)
% Open-loop sinusoidal joint torques.
% After release time, zero torque (arm free to coast).
if t > p.sim.t_release
    tau = zeros(p.N, 1);
else
    tau = amp .* sin(omega_t * t + phi);
end
end


%% ═══════════════════════════════════════════════════════════════════════
%%  Local helper: bool to string
%% ═══════════════════════════════════════════════════════════════════════
function s = bool2str(b)
if b, s = 'YES ⚠'; else, s = 'none'; end
end