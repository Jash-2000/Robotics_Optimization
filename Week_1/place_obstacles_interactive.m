function obstacles = place_obstacles_interactive(p, ax)
% place_obstacles_interactive.m
% =========================================================================
% Interactive obstacle placement in the simulation environment.
% The user clicks in the figure window to place obstacle centres, then
% specifies type and geometry via command-line prompts.
%
% INPUTS:
%   p   – parameter struct from params.m
%   ax  – axes handle of the environment figure (created externally)
%
% OUTPUT:
%   obstacles – struct array, each element has fields:
%       .type    – 'circle' or 'rectangle'
%     For circle:
%       .cx, .cy – centre [m]
%       .r       – radius [m]
%     For rectangle:
%       .cx, .cy – centre [m]
%       .hw      – half-width  (x-extent) [m]
%       .hh      – half-height (y-extent) [m]
%       .phi     – orientation angle [rad] (0 = axis-aligned)
%
% USAGE:
%   Run AFTER the environment axes have been drawn by draw_environment().
%   The function loops until the user presses Enter without clicking.
% =========================================================================

fprintf('\n=== Interactive Obstacle Placement ===\n');
fprintf('  Instructions:\n');
fprintf('  1. Click in the figure window to place an obstacle centre.\n');
fprintf('  2. Answer the command-line prompts for type and size.\n');
fprintf('  3. Press ENTER (without clicking) to finish placing.\n\n');

obstacles = struct('type',{},'cx',{},'cy',{},'r',{},'hw',{},'hh',{},'phi',{});
obs_count = 0;

axes(ax);   % make sure we are operating on the correct axes

while true
    fprintf('  Click to place obstacle %d  (or press ENTER to finish): ', ...
        obs_count + 1);

    % Wait for a mouse click; detect ENTER key via button=0 trick
    try
        [cx, cy, button] = ginput(1);
    catch
        % Figure was closed
        break;
    end

    if isempty(button) || button == 13   % ENTER or no click
        fprintf('  Done placing obstacles.\n\n');
        break;
    end

    % Validate placement (inside reasonable world bounds)
    x_min = -0.5;
    x_max = p.task.d + 1.0;
    y_min = 0.0;
    y_max = sum(p.l) + p.y0 + 0.5;

    if cx < x_min || cx > x_max || cy < y_min || cy > y_max
        fprintf('\n  [!] Click outside valid region. Try again.\n\n');
        continue;
    end

    fprintf('\n');

    % Ask for obstacle type
    while true
        type_in = input('  Obstacle type  [c = circle, r = rectangle]: ', 's');
        type_in = strtrim(lower(type_in));
        if strcmp(type_in, 'c') || strcmp(type_in, 'r')
            break;
        end
        fprintf('  Please enter c or r.\n');
    end

    obs_count = obs_count + 1;

    if strcmp(type_in, 'c')
        %% Circle
        radius = input('  Radius [m]: ');
        while ~isnumeric(radius) || radius <= 0
            fprintf('  Radius must be a positive number.\n');
            radius = input('  Radius [m]: ');
        end

        obs.type = 'circle';
        obs.cx   = cx;
        obs.cy   = cy;
        obs.r    = radius;
        obs.hw   = radius;   % for bounding box in collision checks
        obs.hh   = radius;
        obs.phi  = 0;

        % Draw immediately on the environment axes
        theta_circ = linspace(0, 2*pi, 100);
        xc = cx + radius*cos(theta_circ);
        yc = cy + radius*sin(theta_circ);
        fill(ax, xc, yc, p.viz.col.obstacle_c, 'FaceAlpha', 0.5, ...
            'EdgeColor', p.viz.col.obstacle_c*0.7, 'LineWidth', 1.5);
        text(ax, cx, cy, sprintf('C%d', obs_count), ...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold',...
            'Color','white');
        drawnow;

        fprintf('  Circle obstacle C%d placed at (%.2f, %.2f), r=%.3f m\n\n', ...
            obs_count, cx, cy, radius);

    else
        %% Rectangle
        hw  = input('  Half-width  (x-extent) [m]: ');
        hh  = input('  Half-height (y-extent) [m]: ');
        phi = input('  Orientation angle [deg, 0=axis-aligned]: ');
        phi = phi * pi/180;

        while ~isnumeric(hw) || hw <= 0
            fprintf('  Half-width must be positive.\n');
            hw = input('  Half-width  (x-extent) [m]: ');
        end
        while ~isnumeric(hh) || hh <= 0
            fprintf('  Half-height must be positive.\n');
            hh = input('  Half-height (y-extent) [m]: ');
        end

        obs.type = 'rectangle';
        obs.cx   = cx;
        obs.cy   = cy;
        obs.r    = sqrt(hw^2 + hh^2);   % circumradius for fast check
        obs.hw   = hw;
        obs.hh   = hh;
        obs.phi  = phi;

        % Draw rectangle on axes
        corners_local = [ hw,  hh;
                         -hw,  hh;
                         -hw, -hh;
                          hw, -hh;
                          hw,  hh ]';   % 2×5
        R_phi = [cos(phi), -sin(phi); sin(phi), cos(phi)];
        corners_world = R_phi * corners_local;
        fill(ax, cx + corners_world(1,:), cy + corners_world(2,:), ...
            p.viz.col.obstacle_r, 'FaceAlpha', 0.5, ...
            'EdgeColor', p.viz.col.obstacle_r*0.7, 'LineWidth', 1.5);
        text(ax, cx, cy, sprintf('R%d', obs_count), ...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold',...
            'Color','white');
        drawnow;

        fprintf('  Rectangle R%d at (%.2f, %.2f), hw=%.3f, hh=%.3f, phi=%.1f deg\n\n',...
            obs_count, cx, cy, hw, hh, phi*180/pi);
    end

    obstacles(obs_count) = obs;
end

if obs_count == 0
    fprintf('  No obstacles placed. Proceeding without obstacles.\n\n');
end

end
