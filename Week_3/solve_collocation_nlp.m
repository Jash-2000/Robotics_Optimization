function sol = solve_collocation_nlp(nlp, z0, p)
% solve_collocation_nlp.m
% =========================================================================
% Solves the direct collocation NLP using IPOPT via CasADi.
%
% INPUTS:
%   nlp – NLP structure from build_collocation_nlp
%   z0  – initial guess vector
%   p   – parameter struct
%
% OUTPUT:
%   sol – CasADi solution structure with fields:
%     .x     -- optimal solution vector
%     .f     -- optimal objective value
%     .g     -- constraint values at solution
%     .stats -- solver statistics
%
% =========================================================================

import casadi.*

opti = nlp.opti;

%% ── Set initial guess ─────────────────────────────────────────────────────
fprintf('  Setting initial guess...\n');

% Unpack z0 into state, control, and release time
M = nlp.M;
N = nlp.N;

idx_q = 1:(N*M);
idx_qdot = (N*M+1):(2*N*M);
idx_tau = (2*N*M+1):(3*N*M);
idx_t = 3*N*M + 1;

q0_mat = reshape(z0(idx_q), N, M);
qdot0_mat = reshape(z0(idx_qdot), N, M);
tau0_mat = reshape(z0(idx_tau), N, M);
t_release0 = z0(idx_t);

% Set initial values in Opti stack
opti.set_initial(nlp.q, q0_mat);
opti.set_initial(nlp.qdot, qdot0_mat);
opti.set_initial(nlp.tau, tau0_mat);
opti.set_initial(nlp.t_release, t_release0);

fprintf('    Initial guess set for %d variables\n', length(z0));

%% ── Configure IPOPT solver options ───────────────────────────────────────
fprintf('  Configuring IPOPT solver...\n');

opts = struct();
opts.ipopt.max_iter = p.opt.ipopt_max_iter;
opts.ipopt.tol = p.opt.ipopt_tol;
opts.ipopt.print_level = p.opt.ipopt_print_level;
opts.ipopt.sb = 'yes';  % Suppress banner
opts.ipopt.warm_start_init_point = 'yes';
opts.ipopt.mu_strategy = 'adaptive';
opts.ipopt.hessian_approximation = 'limited-memory';  % BFGS approximation

% Linear solver (choose based on problem size)
if nlp.n_vars < 1000
    opts.ipopt.linear_solver = 'mumps';  % Direct solver for small-medium problems
else
    opts.ipopt.linear_solver = 'ma27';   % Sparse direct solver for large problems
end

opti.solver('ipopt', opts);

fprintf('    Solver: IPOPT\n');
fprintf('    Max iterations: %d\n', opts.ipopt.max_iter);
fprintf('    Tolerance: %.1e\n', opts.ipopt.tol);
fprintf('    Hessian: %s\n', opts.ipopt.hessian_approximation);
fprintf('    Linear solver: %s\n', opts.ipopt.linear_solver);

%% ── Solve NLP ─────────────────────────────────────────────────────────────
fprintf('  Solving NLP with IPOPT...\n');
fprintf('  ───────────────────────────────────────────────────────────\n');

try
    sol = opti.solve();
    fprintf('  ───────────────────────────────────────────────────────────\n');
    fprintf('  ✓ IPOPT converged successfully\n');
    
catch ME
    % IPOPT may throw exception even for acceptable solutions
    fprintf('  ───────────────────────────────────────────────────────────\n');
    fprintf('  ⚠ IPOPT terminated with status: %s\n', ME.message);
    
    % Try to get debug solution anyway
    try
        sol = opti.debug;
        fprintf('    Retrieved debug solution (may be suboptimal)\n');
    catch
        fprintf('    Could not retrieve solution\n');
        rethrow(ME);
    end
end

%% ── Extract solution statistics ──────────────────────────────────────────
fprintf('\n');
fprintf('  Solution statistics:\n');
fprintf('    Return status: %s\n', sol.stats.return_status);
fprintf('    Iterations: %d\n', sol.stats.iter_count);
fprintf('    Objective value: %.6e\n', full(sol.value(opti.f)));

% Check constraint satisfaction
if isfield(sol.stats, 'g')
    g_val = full(sol.stats.g);
    fprintf('    Max constraint violation: %.6e\n', max(abs(g_val)));
end

end
