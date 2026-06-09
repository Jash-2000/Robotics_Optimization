function animate_simulation(arm_sol, traj, obstacles, p)
% animate_simulation.m
% =========================================================================
% Produces a smooth animation of:
%   Phase 1 : robotic arm moving (with object held)
%   Phase 2 : object in ballistic flight (arm freezes at release pose)
%
% Interactive controls (Play / Pause / Restart buttons in animation window).
%
% Separate plot figures (opened after animation window):
%   Plot 1 – Arm-only Energy (KE, PE, Total E) vs time
%   Plot 2 – End-effector trajectory (x vs y) + Object trajectory (x vs y)
%   Plot 3 – Joint positions (q_i) vs time for all N joints
%
% INPUTS:
%   arm_sol   – ODE solution struct from ode45 (fields: .t, .y)
%               .y rows 1:N = q, rows N+1:2N = qdot
%   traj      – ballistic trajectory struct from ballistic_trajectory.m
%   obstacles – obstacle struct array from place_obstacles.m
%   p         – parameter struct from params.m
% =========================================================================

N     = p.N;
dt_fr = 1 / p.viz.fps;

%% ── Extract arm solution ─────────────────────────────────────────────────
t_arm   = arm_sol.t;
q_arm   = arm_sol.y(1:N, :);
qd_arm  = arm_sol.y(N+1:2*N, :);

i_rel   = find(t_arm >= p.sim.t_release, 1);
if isempty(i_rel), i_rel = length(t_arm); end
q_rel   = q_arm(:, i_rel);

%% ── Build animation frame time vector ───────────────────────────────────
t_end_anim = t_arm(end) + (traj.landed * traj.t_land) * 1.1;
t_frames   = 0 : dt_fr : t_end_anim;
n_frames   = length(t_frames);

%% ── Compute ARM-ONLY energy (exclude object) ─────────────────────────────
% Re-derive arm-only energies using link parameters only (no obj augment).
% Approach: subtract object contribution from full T_func / V_func.
% Simpler and exact: compute object KE/PE separately and subtract.
n_arm  = length(t_arm);
KE_full = zeros(1, n_arm);
PE_full = zeros(1, n_arm);
KE_obj  = zeros(1, n_arm);
PE_obj  = zeros(1, n_arm);

for k = 1:n_arm
    qk  = q_arm(:,k);
    qdk = qd_arm(:,k);

    % Full system (arm + object) energies from symbolic functions
    KE_full(k) = T_func(qk, qdk);
    PE_full(k) = V_func(qk);

    % Object contribution
    [~, ~, J_obj_lin, J_obj_ang] = jacobians(qk, p);
    v_obj      = J_obj_lin * qdk;
    omega_obj  = J_obj_ang * qdk;
    KE_obj(k)  = 0.5 * p.obj.mass * (v_obj' * v_obj) ...
               + 0.5 * p.obj.I   * omega_obj^2;

    alpha_N   = sum(qk);
    R_N       = [cos(alpha_N),-sin(alpha_N); sin(alpha_N),cos(alpha_N)];
    [~,~,ee]  = forward_kinematics(qk, p);
    p_obj_com = ee + R_N * p.obj.r_gc;
    PE_obj(k) = p.obj.mass * p.g * p_obj_com(2);
end

% Arm-only energies
KE_arr = KE_full - KE_obj;
PE_arr = PE_full - PE_obj;
E_arr  = KE_arr  + PE_arr;

%% ── Pre-compute end-effector trajectory (arm phase only) ─────────────────
ee_traj = zeros(2, n_arm);
for k = 1:n_arm
    [~, ~, ee_traj(:,k)] = forward_kinematics(q_arm(:,k), p);
end

%% ═══════════════════════════════════════════════════════════════════════════
%% FIGURE 1 — Animation window (separate, interactive)
%% ═══════════════════════════════════════════════════════════════════════════
fig = figure('Name','Fig 1 — Arm Animation (Interactive)', ...
    'Position',[30, 80, p.viz.fig_width, p.viz.fig_height], ...
    'Color','white');

%% ── Interactive control state (shared via appdata) ───────────────────────
setappdata(fig, 'playing',  true);
setappdata(fig, 'restart',  false);
setappdata(fig, 'frame_idx', 1);

%% ── UI Buttons ───────────────────────────────────────────────────────────
btn_w = 100;  btn_h = 30;  btn_y = 8;
uicontrol(fig, 'Style','pushbutton', 'String','⏸ Pause', ...
    'Position',[10, btn_y, btn_w, btn_h], 'FontSize',10, ...
    'Callback', @(~,~) setappdata(fig,'playing',false));
uicontrol(fig, 'Style','pushbutton', 'String','▶ Play', ...
    'Position',[115, btn_y, btn_w, btn_h], 'FontSize',10, ...
    'Callback', @(~,~) setappdata(fig,'playing',true));
uicontrol(fig, 'Style','pushbutton', 'String','↺ Restart', ...
    'Position',[220, btn_y, btn_w, btn_h], 'FontSize',10, ...
    'Callback', @(~,~) setappdata(fig,'restart',true));

%% ── Main animation axes (full figure, leave room for buttons at bottom) ──
ax_main = axes(fig, 'Position',[0.05, 0.12, 0.92, 0.85]);
hold(ax_main, 'on');
grid(ax_main, 'on');
ax_main.GridAlpha = 0.15;

arm_reach = sum(p.l);
x_min = -arm_reach - p.viz.axis_margin;
x_max =  p.task.d  + p.viz.axis_margin;
y_min = -0.2;
y_max =  p.y0 + arm_reach + p.viz.axis_margin;
axis(ax_main, [x_min, x_max, y_min, y_max]);
daspect(ax_main, [1 1 1]);
xlabel(ax_main,'x [m]','FontSize',11);
ylabel(ax_main,'y [m]','FontSize',11);
title(ax_main,'4-Link Planar Arm — Throwing Simulation  |  ▶ Play  ⏸ Pause  ↺ Restart','FontSize',11);

% Ground
plot(ax_main, [x_min,x_max],[0,0],'-','Color',p.viz.col.ground,'LineWidth',2.5);
hatch_x = x_min:0.3:x_max;
for hx = hatch_x
    plot(ax_main,[hx,hx+0.15],[0,-0.10],'-','Color',p.viz.col.ground*0.7,'LineWidth',0.7);
end

% Pedestal & base
plot(ax_main,[0,0],[0,p.y0],'-','Color',[0.3 0.3 0.3],'LineWidth',4);
plot(ax_main,0,p.y0,'s','MarkerSize',13,'MarkerFaceColor',[0.3 0.3 0.3],'MarkerEdgeColor','k');

% Target
plot(ax_main,p.task.d,0,'p','MarkerSize',20,'MarkerFaceColor',p.viz.col.target,...
    'MarkerEdgeColor',p.viz.col.target*0.6,'LineWidth',1.5);
text(ax_main,p.task.d,0.15,sprintf('Target\n(%.2f m)',p.task.d),...
    'HorizontalAlignment','center','FontSize',9,'Color',p.viz.col.target,'FontWeight','bold');

% Ballistic trajectory arc (drawn in advance, thin)
plot(ax_main, traj.pos(1,:), traj.pos(2,:), '--', ...
    'Color', p.viz.col.trajectory, 'LineWidth', 1.2);

% Obstacles
draw_obstacles_on_axes(ax_main, obstacles, p);

%% ── Animated graphics objects (pre-allocate handles) ────────────────────
% Arm links
h_links = gobjects(N, 1);
for i = 1:N
    col = p.viz.col.link;
    lw  = p.viz.arm_lw;
    if i == N
        col = p.viz.col.gripper;
        lw  = p.viz.arm_lw - 1;
    end
    h_links(i) = plot(ax_main, [0 0],[0 0],'-','Color',col,'LineWidth',lw);
end
% Joint markers
h_joints = plot(ax_main, zeros(1,N+1), zeros(1,N+1), 'o', ...
    'MarkerSize',6,'MarkerFaceColor','k','MarkerEdgeColor','k');
% Object bounding shape
h_obj = [];   % created dynamically based on shape

% Time stamp text
h_time = text(ax_main, x_min+0.1, y_max-0.1, '', 'FontSize',10, ...
    'Color',[0.2 0.2 0.2],'FontWeight','bold');

% Collision warning text
h_warn = text(ax_main, (x_min+x_max)/2, y_max-0.2, '', ...
    'FontSize',13,'Color',[0.85 0.1 0.1],'FontWeight','bold',...
    'HorizontalAlignment','center','Visible','off');

%% ── Animation loop (interactive: Play / Pause / Restart) ─────────────────
fprintf('  Animating ... Use Play/Pause/Restart buttons in figure.\n');
fprintf('  Close the animation figure to proceed to plots.\n\n');

fr = 1;
collision_has_occurred = false;
while ishandle(fig)

    % ── Restart ────────────────────────────────────────────────────────
    if getappdata(fig,'restart')
        fr = 1;
        collision_has_occurred = false;
        setappdata(fig,'restart', false);
        setappdata(fig,'playing', true);

        % Clear landing marker if drawn
        delete(findobj(ax_main,'Tag','landing_marker'));

        set(h_warn,'Visible','off');
    end

    % ── Pause: spin-wait without advancing frame ────────────────────────
    if ~getappdata(fig,'playing')
        pause(0.05);
        continue;
    end

    % ── End of animation: pause and wait for Restart or close ──────────
    if fr > n_frames
        setappdata(fig,'playing', false);
        pause(0.05);
        continue;
    end

    t_now = t_frames(fr);

    %% Arm state
    if t_now <= p.sim.t_release
        q_now  = interp1(t_arm', q_arm',  t_now)';
        phase  = 1;
    else
        q_now  = q_rel;
        phase  = 2;
    end

    %% FK
    [joint_xy, ~, ~] = forward_kinematics(q_now, p);
    for i = 1:N
        set(h_links(i), 'XData', joint_xy(1,[i,i+1]), ...
                         'YData', joint_xy(2,[i,i+1]));
    end
    set(h_joints, 'XData', joint_xy(1,:), 'YData', joint_xy(2,:));

    %% Object position & orientation
    if phase == 1
        alpha_N = sum(q_now);
        R_N     = [cos(alpha_N),-sin(alpha_N); sin(alpha_N),cos(alpha_N)];
        obj_pos = joint_xy(:,end) + R_N * p.obj.r_gc;
        obj_th  = alpha_N;
    else
        t_flight = min(t_now - p.sim.t_release, traj.t(end));
        obj_pos  = traj.r0 + traj.v0*t_flight + 0.5*[0;-p.g]*t_flight^2;
        obj_th   = sum(q_rel) + traj.omega * t_flight;
    end

    if ~isempty(h_obj) && ishandle(h_obj), delete(h_obj); end
    h_obj = draw_object_shape(ax_main, obj_pos, obj_th, p);

    %% Collision check
    % any_collision = false;
    % if ~isempty(obstacles)
    %     for i = 1:N
    %         ql.type = 'link'; ql.p1 = joint_xy(:,i); ql.p2 = joint_xy(:,i+1);
    %         [ci,~] = check_collision(ql, obstacles, p);
    %         if ci, any_collision = true; break; end
    %     end
    %     qo.type = 'object'; qo.pos = obj_pos; qo.theta = obj_th;
    %     [co,~] = check_collision(qo, obstacles, p);
    %     if co, any_collision = true; end
    % end
    % 
    % 
    % if any_collision
    %     set(h_warn,'String','⚠ COLLISION','Visible','on');
    % else
    %     set(h_warn,'Visible','off');
    % end

    any_collision = false;
    
    if ~isempty(obstacles)
        for i = 1:N
            ql.type = 'link';
            ql.p1 = joint_xy(:,i);
            ql.p2 = joint_xy(:,i+1);
    
            [ci,~] = check_collision(ql, obstacles, p);
    
            if ci
                any_collision = true;
                break;
            end
        end
    
        qo.type  = 'object';
        qo.pos   = obj_pos;
        qo.theta = obj_th;
    
        [co,~] = check_collision(qo, obstacles, p);
    
        if co
            any_collision = true;
        end
    end
    
    % Latch collision once it happens
    if any_collision
        collision_has_occurred = true;
    end
    
    % Keep warning visible until animation ends/restarts
    if collision_has_occurred
        set(h_warn,'String','⚠ COLLISION DETECTED','Visible','on');
    else
        set(h_warn,'Visible','off');
    end

    %% Time stamp
    set(h_time,'String', sprintf('t = %.2f s  |  Phase %d', t_now, phase));

    %% Landing marker (drawn once when object hits ground)
    if traj.landed && phase == 2 && ...
       (t_now - p.sim.t_release) >= traj.t_land && ...
       isempty(findobj(ax_main,'Tag','landing_marker'))
        plot(ax_main, traj.x_land, 0, 'x', 'MarkerSize',16, ...
            'MarkerEdgeColor',[0.85 0.1 0.1],'LineWidth',3, ...
            'Tag','landing_marker');
        text(ax_main, traj.x_land, 0.18, sprintf('Land: %.3f m',traj.x_land),...
            'HorizontalAlignment','center','FontSize',9,'Color',[0.85 0.1 0.1]);
    end

    drawnow limitrate;
    pause(dt_fr);

    fr = fr + 1;
end

fprintf('  Animation window closed. Generating plots ...\n\n');

%% ═══════════════════════════════════════════════════════════════════════════
%% PLOT 1 — Arm-only Energy (KE, PE, Total) vs time
%% ═══════════════════════════════════════════════════════════════════════════
fig1 = figure('Name','Plot 1 — Arm Energy', ...
    'Position',[50, 80, 750, 420], 'Color','white');
ax1 = axes(fig1);
hold(ax1,'on'); grid(ax1,'on');

plot(ax1, t_arm, KE_arr, 'b-',  'LineWidth',2.0, 'DisplayName','KE (arm)');
plot(ax1, t_arm, PE_arr, 'r--', 'LineWidth',2.0, 'DisplayName','PE (arm)');
plot(ax1, t_arm, E_arr,  'k-',  'LineWidth',2.5, 'DisplayName','Total E (arm)');
xline(ax1, p.sim.t_release, '--', 'Color',[0.5 0.5 0.5], 'LineWidth',1.5,...
    'Label','Release','LabelVerticalAlignment','bottom','FontSize',9);

xlabel(ax1,'Time  [s]','FontSize',12);
ylabel(ax1,'Energy  [J]','FontSize',12);
title(ax1,'Arm-Only System Energy  (object contribution excluded)','FontSize',12);
legend(ax1,'Location','best','FontSize',10);
ax1.FontSize = 11;

%% ═══════════════════════════════════════════════════════════════════════════
%% PLOT 2 — End-effector trajectory + Object trajectory (x vs y)
%% ═══════════════════════════════════════════════════════════════════════════
fig2 = figure('Name','Plot 2 — Trajectories (x vs y)', ...
    'Position',[820, 80, 750, 500], 'Color','white');
ax2 = axes(fig2);
hold(ax2,'on'); grid(ax2,'on');
daspect(ax2,[1 1 1]);

% Ground line
x_lo = min([ee_traj(1,:), traj.pos(1,:)]) - 0.2;
x_hi = max([ee_traj(1,:), traj.pos(1,:), p.task.d]) + 0.3;
plot(ax2,[x_lo,x_hi],[0,0],'-','Color',p.viz.col.ground,'LineWidth',2);

% End-effector path (arm phase)
plot(ax2, ee_traj(1,:), ee_traj(2,:), '-', ...
    'Color', p.viz.col.link, 'LineWidth', 2.0, ...
    'DisplayName','End-effector path');
% Mark release point
plot(ax2, ee_traj(1,i_rel), ee_traj(2,i_rel), 'o', ...
    'MarkerSize',9,'MarkerFaceColor','w','MarkerEdgeColor',p.viz.col.link,...
    'LineWidth',2,'DisplayName','Release point');

% Object CoM trajectory (ballistic)
plot(ax2, traj.pos(1,:), traj.pos(2,:), '--', ...
    'Color', p.viz.col.trajectory, 'LineWidth', 2.0, ...
    'DisplayName','Object CoM (ballistic)');

% Target marker
plot(ax2, p.task.d, 0, 'p', 'MarkerSize',18, ...
    'MarkerFaceColor',p.viz.col.target,'MarkerEdgeColor',p.viz.col.target*0.6,...
    'DisplayName',sprintf('Target (%.2f m)',p.task.d));

% Landing point
if traj.landed
    plot(ax2, traj.x_land, 0, 'x', 'MarkerSize',14, ...
        'MarkerEdgeColor',[0.85 0.1 0.1],'LineWidth',2.5,...
        'DisplayName',sprintf('Landing (%.3f m)',traj.x_land));
end

% Obstacles (on trajectory plot for spatial context)
draw_obstacles_on_axes(ax2, obstacles, p);

xlabel(ax2,'x  [m]','FontSize',12);
ylabel(ax2,'y  [m]','FontSize',12);
title(ax2,'End-Effector & Object Trajectories','FontSize',12);
legend(ax2,'Location','best','FontSize',10);
ax2.FontSize = 11;

%% ═══════════════════════════════════════════════════════════════════════════
%% PLOT 3 — Joint positions vs time
%% ═══════════════════════════════════════════════════════════════════════════
fig3 = figure('Name','Plot 3 — Joint Positions vs Time', ...
    'Position',[50, 560, 1200, 400], 'Color','white');

joint_labels = {'Joint 1 (shoulder)','Joint 2','Joint 3','Joint 4 (gripper)'};
colors_j = lines(N);

for i = 1:N
    ax_j = subplot(1, N, i, 'Parent', fig3);
    hold(ax_j,'on'); grid(ax_j,'on');

    % Convert radians to degrees for readability
    plot(ax_j, t_arm, rad2deg(q_arm(i,:)), '-', ...
        'Color', colors_j(i,:), 'LineWidth', 2.0);
    xline(ax_j, p.sim.t_release, '--','Color',[0.5 0.5 0.5],'LineWidth',1.2,...
        'Label','Release','LabelVerticalAlignment','bottom','FontSize',8);

    xlabel(ax_j,'Time  [s]','FontSize',11);
    if i == 1
        ylabel(ax_j,'Angle  [deg]','FontSize',11);
    end
    title(ax_j, joint_labels{i}, 'FontSize',11);
    ax_j.FontSize = 10;
end
sgtitle(fig3,'Joint Positions vs Time  (relative angles)','FontSize',13);

fprintf('  Plots generated:\n');
fprintf('    Fig 1 — Arm energy (KE / PE / Total)\n');
fprintf('    Fig 2 — End-effector + object trajectories\n');
fprintf('    Fig 3 — Joint positions vs time\n\n');

end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: draw object bounding shape
%% ═══════════════════════════════════════════════════════════════════════════
function h = draw_object_shape(ax, pos, theta, p)
col   = p.viz.col.object;
switch p.obj.shape
    case 'circle'
        th = linspace(0,2*pi,60);
        xc = pos(1) + p.obj.r*cos(th);
        yc = pos(2) + p.obj.r*sin(th);
        h  = fill(ax, xc, yc, col, 'FaceAlpha',0.7, ...
            'EdgeColor',col*0.6,'LineWidth',1.2);
    case 'rectangle'
        corners = [ p.obj.a,  p.obj.b;
                   -p.obj.a,  p.obj.b;
                   -p.obj.a, -p.obj.b;
                    p.obj.a, -p.obj.b;
                    p.obj.a,  p.obj.b ]';
        R = [cos(theta),-sin(theta); sin(theta),cos(theta)];
        cw = R*corners;
        h  = fill(ax, pos(1)+cw(1,:), pos(2)+cw(2,:), col, ...
            'FaceAlpha',0.7,'EdgeColor',col*0.6,'LineWidth',1.2);
    otherwise
        h = plot(ax, pos(1), pos(2), 'o','MarkerSize',8,...
            'MarkerFaceColor',col,'MarkerEdgeColor',col*0.6);
end
end


%% ═══════════════════════════════════════════════════════════════════════════
%%  Helper: draw obstacles on given axes
%% ═══════════════════════════════════════════════════════════════════════════
function draw_obstacles_on_axes(ax, obstacles, p)
if isempty(obstacles), return; end
for k = 1:numel(obstacles)
    obs = obstacles(k);
    switch obs.type
        case 'circle'
            th = linspace(0,2*pi,100);
            fill(ax, obs.cx+obs.r*cos(th), obs.cy+obs.r*sin(th), ...
                p.viz.col.obstacle_c,'FaceAlpha',0.5,...
                'EdgeColor',p.viz.col.obstacle_c*0.7,'LineWidth',1.5);
        case 'rectangle'
            corners = [ obs.hw,  obs.hh;
                       -obs.hw,  obs.hh;
                       -obs.hw, -obs.hh;
                        obs.hw, -obs.hh;
                        obs.hw,  obs.hh]';
            R = [cos(obs.phi),-sin(obs.phi); sin(obs.phi),cos(obs.phi)];
            cw = R*corners;
            fill(ax, obs.cx+cw(1,:), obs.cy+cw(2,:), ...
                p.viz.col.obstacle_r,'FaceAlpha',0.5,...
                'EdgeColor',p.viz.col.obstacle_r*0.7,'LineWidth',1.5);
    end
end
end