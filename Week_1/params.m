function p = params()
% params.m
% =========================================================================
% Central parameter definition for the 4-link planar robotic arm throwing
% simulation. ALL user-tunable quantities live here. Edit this file to
% change the robot, object, task, or simulation settings.
%
% Coordinate convention
%   x : horizontal (positive = away from base, toward target)
%   y : vertical   (positive = up)
%   Origin : base joint of the robot
%   Ground : y = 0
%   Robot base is LIFTED to y = y0 (e.g. mounted on a table)
%
% Joint angle convention
%   q_i : angle of link i measured from the PREVIOUS link's axis
%          (relative / local joint angle)
%   Initial pose : all links pointing straight up => q = [pi/2; 0; 0; 0]
%          Link 1 makes 90 deg with horizontal base; remaining links
%          are aligned with link 1 (zero relative angle = fully extended)
% =========================================================================

%% ── Robot geometry ──────────────────────────────────────────────────────
p.N   = 4;                      % number of links (3 long + 1 gripper)

% Link lengths [m]  (link 4 is short gripper link)
p.l   = [0.40; 0.35; 0.30; 0.08];

% Link masses [kg]
p.m   = [2.0;  1.5;  1.0;  0.3];

% Link centre-of-mass offset from proximal joint [m]
% (uniform rod => lc_i = l_i / 2)
p.lc  = p.l / 2;

% Link moments of inertia about link CoM [kg·m²]
% (thin rod rotating about CoM: I = (1/12)*m*l^2)
p.I   = (1/12) .* p.m .* p.l.^2;

% Base height above ground [m]  (robot mounted on table / pedestal)
p.y0  = 1.0;

%% ── Object (held in gripper at t = 0) ───────────────────────────────────
% Shape: 'circle' or 'rectangle'
p.obj.shape   = 'rectangle';

% For 'circle'    : radius r  [m]
p.obj.r       = 0.08;

% For 'rectangle' : half-lengths along object-local x and y axes [m]
p.obj.a       = 0.12;           % half-length (along long axis)
p.obj.b       = 0.06;           % half-width

% Mass [kg]
p.obj.mass    = 0.5;

% Moment of inertia about object CoM [kg·m²]
%   Rectangle : (1/12)*m*(4a^2 + 4b^2) = (m/3)*(a^2+b^2)
p.obj.I       = (p.obj.mass / 3) * (p.obj.a^2 + p.obj.b^2);

% Gripper-to-CoM offset vector in GRIPPER (link-4) body frame [m]
%   Positive x4 points along link 4 away from joint 4
%   Typically the object CoM is ahead of the gripper tip along link axis
p.obj.r_gc    = [0.05; 0.00];   % [x4_body; y4_body]

%% ── Task ─────────────────────────────────────────────────────────────────
% Target location: on the ground (y=0) at horizontal distance d from base
p.task.d      = 2.977;          % [m]  horizontal distance to target

% Gravitational acceleration [m/s²]
p.g           = 9.81;

%% ── Simulation ───────────────────────────────────────────────────────────
p.sim.t_end      = 3.0;         % total simulation time [s]
p.sim.dt         = 0.01;        % output time step for ODE [s]
p.sim.ode_opts   = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',1e-3);

% Release time (Week 1: user-specified; later weeks: optimized)
p.sim.t_release  = 1.2;        % [s] time at which object is released

% Obstacle mode: 'fixed' (use 3 predefined configs) or 'interactive' (user clicks)
p.sim.obstacle_mode = 'fixed';  % 'fixed' | 'interactive'

%% ── Initial joint configuration ─────────────────────────────────────────
% Fully upright: link 1 perpendicular to horizontal base line.
% Absolute angle of link 1 from horizontal = pi/2.
% All remaining relative angles = 0 (links aligned = fully extended upward).
p.q0     = [pi/2; 0; 0; 0];    % [rad]  initial joint angles
p.qdot0  = [0; 0; 0; 0];       % [rad/s] initial joint velocities

%% ── Joint and Torque Limits (for Week 2+ optimization) ──────────────────
% Joint angle limits [rad]
p.lim.q_min   = -pi * ones(p.N, 1);
p.lim.q_max   =  pi * ones(p.N, 1);

% Joint velocity limits [rad/s]
p.lim.qd_min  = -5 * ones(p.N, 1);
p.lim.qd_max  =  5 * ones(p.N, 1);

% Joint torque limits [N·m]
% Larger joints can produce more torque; gripper (joint 4) is weakest
p.lim.tau_min = [-50; -20; -15; -8];
p.lim.tau_max = [ 50;  20;  15;  8];

%% ── Fixed Obstacle Configurations (for reproducible testing) ────────────
% Configuration 1: Simple (baseline)
% Purpose: Verify all methods converge in weakly constrained environment
% Expected: All three methods converge; miss distance ~5-15 cm
p.configs.simple(1).type = 'circle';
p.configs.simple(1).cx   = 1.3;     % x = 1.3 m (55% of target distance)
p.configs.simple(1).cy   = 0.9;     % y = 0.9 m (below arm base, easy to clear)
p.configs.simple(1).r    = 0.20;    % radius = 20 cm
p.configs.simple(1).hw   = 0.20;    % half-width (for bounding box)
p.configs.simple(1).hh   = 0.20;    % half-height
p.configs.simple(1).phi  = 0;       % orientation (N/A for circles)

% Configuration 2: Moderate (corridor)
% Purpose: Show method robustness when trajectory must navigate obstacles
% Expected: Single-shooting struggles; collocation robust; indirect fastest
p.configs.moderate(1).type = 'circle';
p.configs.moderate(1).cx   = 1.0;   % x = 1.0 m (42% of target distance)
p.configs.moderate(1).cy   = 1.1;   % y = 1.2 m (above arm base)
p.configs.moderate(1).r    = 0.25;  % radius = 25 cm
p.configs.moderate(1).hw   = 0.25;
p.configs.moderate(1).hh   = 0.25;
p.configs.moderate(1).phi  = 0;

p.configs.moderate(2).type = 'circle';
p.configs.moderate(2).cx   = 1.7;   % x = 1.7 m (72% of target distance)
p.configs.moderate(2).cy   = 1.8;   % y = 1.8 m (higher, narrows corridor)
p.configs.moderate(2).r    = 0.25;  % radius = 25 cm
p.configs.moderate(2).hw   = 0.25;
p.configs.moderate(2).hh   = 0.25;
p.configs.moderate(2).phi  = 0;

% Configuration 3: Hard (multiple barriers)
% Purpose: Test convergence under tight constraints; reveal failure modes
% Expected: Single-shooting may fail; collocation and indirect diverge
p.configs.hard(1).type = 'circle';
p.configs.hard(1).cx   = 0.9;       % large, low obstacle
p.configs.hard(1).cy   = 0.8;
p.configs.hard(1).r    = 0.30;      % radius = 30 cm
p.configs.hard(1).hw   = 0.30;
p.configs.hard(1).hh   = 0.30;
p.configs.hard(1).phi  = 0;

p.configs.hard(2).type = 'circle';
p.configs.hard(2).cx   = 1.4;       % medium height
p.configs.hard(2).cy   = 1.6;
p.configs.hard(2).r    = 0.20;      % radius = 20 cm
p.configs.hard(2).hw   = 0.20;
p.configs.hard(2).hh   = 0.20;
p.configs.hard(2).phi  = 0;

p.configs.hard(3).type = 'circle';
p.configs.hard(3).cx   = 1.9;       % high, blocks steep paths
p.configs.hard(3).cy   = 2.2;
p.configs.hard(3).r    = 0.25;      % radius = 25 cm
p.configs.hard(3).hw   = 0.25;
p.configs.hard(3).hh   = 0.25;
p.configs.hard(3).phi  = 0;

% Empty configuration (no obstacles, for debugging)
p.configs.none = struct('type',{},'cx',{},'cy',{},'r',{},'hw',{},'hh',{},'phi',{});

%% ── Visualisation ────────────────────────────────────────────────────────
p.viz.fig_width   = 1200;       % figure pixel width
p.viz.fig_height  = 700;        % figure pixel height
p.viz.arm_lw      = 4;          % arm link line width
p.viz.fps         = 30;         % animation frames per second
p.viz.axis_margin = 0.4;        % extra margin around workspace [m]

% Colours
p.viz.col.link        = [0.20  0.45  0.70];
p.viz.col.gripper     = [0.85  0.33  0.10];
p.viz.col.object      = [0.47  0.67  0.19];
p.viz.col.trajectory  = [0.93  0.69  0.13];
p.viz.col.target      = [0.85  0.10  0.10];
p.viz.col.ground      = [0.40  0.40  0.40];
p.viz.col.obstacle_c  = [0.75  0.22  0.17];   % circle obstacle
p.viz.col.obstacle_r  = [0.49  0.18  0.56];   % rectangle obstacle

p.opt.M = 60;  % Number of collocation points
p.opt.collision_margin = 0.0;  % Safety margin [m]

% Multi-objective weights
% KEY INSIGHT: Collision avoidance MUST dominate all other objectives.
% The optimizer should NEVER accept a collision to improve landing accuracy.
p.opt.w_position = 1000;    % Reduced from 1000 (still important but not dominant)
p.opt.w_energy = 0.1;      % Reduced (just regularization, not a real objective)
p.opt.w_direction = 1.0;     % Reduced (soft preference, not critical)

% Release time bounds
p.opt.t_release_min = 0.8;   % Minimum release time [s]
p.opt.t_release_max = 2.0;   % Maximum release time [s]

% NLP solver options
p.opt.ipopt_max_iter = 5000;    % More iterations for complex problem
p.opt.ipopt_tol = 1e-5;
p.opt.ipopt_print_level = 5;


end