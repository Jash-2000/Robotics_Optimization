function [accept, rho_new, rho_u_new, tr_info] = ...
    scp_trust_region_update(J_old, J_new, J_pred, rho, rho_u, scp)
% scp_trust_region_update.m
% =========================================================================
% Decide whether to accept the SCP step and update the trust-region radius.
%
% CRITICAL RULE: NEVER accept a step that increases actual cost.
%   J_new >= J_old  →  always reject (regardless of trust ratio).
%
% Trust ratio (only meaningful when J_new < J_old):
%   xi = (J_old - J_new) / (J_old - J_pred)
%
%   xi > 0.75     →  accept, expand ρ   (model is very accurate)
%   0.25 < xi     →  accept, keep ρ     (model is decent)
%   0 < xi ≤ 0.25 →  accept, contract ρ (model is poor but improving)
%   xi ≤ 0        →  reject, contract ρ (model is misleading)
% =========================================================================

expand_factor   = 1.5;
contract_factor = 0.5;

tr_info = struct();

% ── HARD RULE: never accept cost increase ────────────────────────────
if J_new >= J_old - 1e-10
    accept    = false;
    rho_new   = max(rho   * contract_factor, scp.trust_radius_min);
    rho_u_new = max(rho_u * contract_factor, scp.trust_radius_u_min);
    tr_info.xi     = NaN;
    tr_info.action = 'REJECT (cost increased)';
    return;
end

% ── Compute trust ratio ─────────────────────────────────────────────
denom = J_old - J_pred;
numer = J_old - J_new;   % guaranteed > 0 here

if abs(denom) < 1e-12
    xi = 1.0;   % model predicted no change, but we improved → treat as good
else
    xi = numer / denom;
end

tr_info.xi = xi;

% ── Decision logic ──────────────────────────────────────────────────
if xi > 0.75
    accept    = true;
    rho_new   = min(rho   * expand_factor, scp.trust_radius_max);
    rho_u_new = min(rho_u * expand_factor, scp.trust_radius_u_max);
    tr_info.action = 'accept + expand';

elseif xi > 0.25
    accept    = true;
    rho_new   = rho;
    rho_u_new = rho_u;
    tr_info.action = 'accept';

elseif xi > 0
    accept    = true;
    rho_new   = max(rho   * contract_factor, scp.trust_radius_min);
    rho_u_new = max(rho_u * contract_factor, scp.trust_radius_u_min);
    tr_info.action = 'accept + contract';

else
    accept    = false;
    rho_new   = max(rho   * contract_factor, scp.trust_radius_min);
    rho_u_new = max(rho_u * contract_factor, scp.trust_radius_u_min);
    tr_info.action = 'REJECT (bad model)';
end

end