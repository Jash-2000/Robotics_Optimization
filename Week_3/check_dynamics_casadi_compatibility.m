% check_dynamics_casadi_compatibility.m
% ===========================================================================
% Checks whether Week 1 dynamics functions (M_func, C_func, G_func) are
% compatible with CasADi symbolic variables.
% ===========================================================================

clear; close all; clc;

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  DYNAMICS CASADI COMPATIBILITY CHECK\n');
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');

%% ── Setup ─────────────────────────────────────────────────────────────────
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');  % Week 1 functions

try
    import casadi.*
    fprintf('[1/4] CasADi loaded\n');
catch ME
    fprintf('[1/4] ✗ CasADi not available: %s\n', ME.message);
    return;
end

%% ── Load parameters ───────────────────────────────────────────────────────
p = params();
fprintf('[2/4] Parameters loaded\n');

%% ── Check if dynamics files exist ─────────────────────────────────────────
if ~exist('M_func.m', 'file')
    fprintf('[3/4] ⚠ Dynamics not generated. Running derive_dynamics...\n');
    derive_dynamics(p);
    fprintf('      ✓ Dynamics generated\n');
else
    fprintf('[3/4] ✓ Dynamics files found\n');
end

%% ── Test CasADi compatibility ─────────────────────────────────────────────
fprintf('[4/4] Testing CasADi compatibility...\n');

% Create symbolic variables (correct CasADi syntax)
q_sym = casadi.SX.sym('q', 4, 1);
qdot_sym = casadi.SX.sym('qdot', 4, 1);

test_passed = true;

% Test M_func
fprintf('      Testing M_func with CasADi symbolic...\n');
try
    M_mat = M_func(q_sym);  % M_func only takes q
    
    % Check if output is symbolic
    if isa(M_mat, 'casadi.SX') || isa(M_mat, 'casadi.MX')
        fprintf('        ✓ M_func returns CasADi symbolic (%s)\n', class(M_mat));
    else
        fprintf('        ✗ M_func returns %s (expected CasADi type)\n', class(M_mat));
        test_passed = false;
    end
    
    % Check dimensions
    if all(size(M_mat) == [4, 4])
        fprintf('        ✓ M_func dimensions correct: [4×4]\n');
    else
        fprintf('        ✗ M_func dimensions wrong: [%d×%d]\n', size(M_mat));
        test_passed = false;
    end
catch ME
    fprintf('        ✗ M_func FAILED: %s\n', ME.message);
    test_passed = false;
end

% Test C_func
fprintf('      Testing C_func with CasADi symbolic...\n');
try
    C_mat = C_func(q_sym, qdot_sym);  % C_func takes q and qdot only
    
    if isa(C_mat, 'casadi.SX') || isa(C_mat, 'casadi.MX')
        fprintf('        ✓ C_func returns CasADi symbolic (%s)\n', class(C_mat));
    else
        fprintf('        ✗ C_func returns %s (expected CasADi type)\n', class(C_mat));
        test_passed = false;
    end
    
    if all(size(C_mat) == [4, 4])
        fprintf('        ✓ C_func dimensions correct: [4×4]\n');
    else
        fprintf('        ✗ C_func dimensions wrong: [%d×%d]\n', size(C_mat));
        test_passed = false;
    end
catch ME
    fprintf('        ✗ C_func FAILED: %s\n', ME.message);
    test_passed = false;
end

% Test G_func
fprintf('      Testing G_func with CasADi symbolic...\n');
try
    G_vec = G_func(q_sym);  % G_func only takes q
    
    if isa(G_vec, 'casadi.SX') || isa(G_vec, 'casadi.MX')
        fprintf('        ✓ G_func returns CasADi symbolic (%s)\n', class(G_vec));
    else
        fprintf('        ✗ G_func returns %s (expected CasADi type)\n', class(G_vec));
        test_passed = false;
    end
    
    % G should be [4×1]
    if (size(G_vec, 1) == 4 && size(G_vec, 2) == 1) || ...
       (size(G_vec, 1) == 1 && size(G_vec, 2) == 4)
        fprintf('        ✓ G_func dimensions correct: [%d×%d]\n', size(G_vec));
    else
        fprintf('        ✗ G_func dimensions wrong: [%d×%d]\n', size(G_vec));
        test_passed = false;
    end
catch ME
    fprintf('        ✗ G_func FAILED: %s\n', ME.message);
    test_passed = false;
end
%% ── Summary ───────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

if test_passed
    fprintf('  ✓ ALL TESTS PASSED\n');
    fprintf('  Week 1 dynamics functions are CasADi-compatible!\n');
    fprintf('  You can use M_func, C_func, G_func directly in Week 3.\n');
else
    fprintf('  ✗ COMPATIBILITY ISSUES DETECTED\n');
    fprintf('  Week 1 dynamics functions may not work with CasADi.\n');
    fprintf('  You may need to regenerate them or create CasADi versions.\n');
end

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('\n');