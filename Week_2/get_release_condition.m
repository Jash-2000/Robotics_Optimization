function rc = get_release_condition(z, p)
% get_release_condition.m
% =========================================================================
% Helper function to simulate trajectory with given decision variables z
% and return the release condition struct.
%
% INPUTS:
%   z – [4M×1] decision variables (piecewise-constant torques)
%   p – parameter struct from params.m
%
% OUTPUT:
%   rc – release condition struct (from release_condition.m) with fields:
%        .obj_pos, .obj_vel, .obj_theta, .obj_omega, .speed, .angle_deg,
%        .t_land, .x_land, .miss, .miss_abs
%
% USAGE:
%   Used by main_week2.m to extract final optimized results for reporting.
% =========================================================================

%% ── Unpack torques and release time ───────────────────────────────────────
[tau_func, t_release] = unpack_torques(z, p);

%% ── Integrate ODE ─────────────────────────────────────────────────────────
q0 = p.q0;
qdot0 = p.qdot0;
y0 = [q0; qdot0];

tspan = [0, t_release];  % Use extracted release time
ode_opts = p.sim.ode_opts;
ode_func = @(t, y) arm_ode(t, y, tau_func, p);

try
    [~, y_vec] = ode45(ode_func, tspan, y0, ode_opts);
    
    % Extract final state
    q_rel = y_vec(end, 1:p.N)';
    qdot_rel = y_vec(end, p.N+1:end)';
    
    % Compute release condition
    rc = release_condition(q_rel, qdot_rel, p);
    
catch ME
    % Integration failed
    fprintf('  ⚠ get_release_condition: ODE integration failed: %s\n', ME.message);
    rc.x_land = NaN;
    rc.miss = NaN;
    rc.miss_abs = NaN;
end

end