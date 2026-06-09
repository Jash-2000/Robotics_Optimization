function [c, ceq] = shooting_constraints(z, p)
% shooting_constraints.m
% =========================================================================
% Constraint function for single-shooting optimization with fmincon.
% Enforces collision avoidance for both manipulation and ballistic phases.
%
% INPUTS:
%   z – [4M×1] decision variables (piecewise-constant torques)
%   p – parameter struct from params.m (includes p.obstacles, p.opt.collision_margin)
%
% OUTPUTS:
%   c   – [n_total×1] inequality constraints (c ≤ 0 enforced by fmincon)
%         Each element is a NEGATIVE signed distance (penetration depth)
%         Constraint satisfied if c(i) ≤ 0 → dist(i) ≥ 0 (no collision)
%   ceq – [] (no equality constraints for this problem)
%
% LOGIC:
%   1. Call shooting_objective(z, p) to get arm_coll and flight_coll
%   2. Convert signed distances to constraint form:
%      - Signed distance d: positive = clearance, negative = penetration
%      - Constraint c = -d + margin: satisfied if c ≤ 0 ⟹ d ≥ margin
%   3. Optional safety margin (default 0.0 m, can set p.opt.collision_margin)
%
% EFFICIENCY NOTE:
%   This approach re-computes objective function. For efficiency, one could
%   store collision info in a persistent variable or use nested function,
%   but this is simpler and fmincon's finite-difference gradient needs
%   constraint re-evaluation anyway.
% =========================================================================

% Get collision info from objective function
[~, arm_coll, flight_coll] = shooting_objective(z, p);

% Safety margin (optional, default = 0.0 m)
if isfield(p.opt, 'collision_margin')
    margin = p.opt.collision_margin;
else
    margin = 0.0;
end

%% ── Inequality constraints: c ≤ 0 ────────────────────────────────────────
% Signed distance convention:
%   dist > 0 → clearance (no collision)
%   dist < 0 → penetration (collision)
%
% Constraint form: c = -dist + margin ≤ 0
%   ⟹ dist ≥ margin (enforces safety buffer)

c_arm = [];
c_flight = [];

if ~isempty(arm_coll)
    c_arm = -arm_coll + margin;  % [n_arm×1]
end

if ~isempty(flight_coll)
    c_flight = -flight_coll + margin;  % [n_flight×1]
end

% Combine all inequality constraints
c = [c_arm; c_flight];

% If objective failed (returned empty), create large violation
if isempty(c)
    c = 1e3;  % Large positive value → constraint violated
end

%% ── Equality constraints (none for this problem) ─────────────────────────
ceq = [];

end
