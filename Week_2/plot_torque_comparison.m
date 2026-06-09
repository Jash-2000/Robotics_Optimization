function plot_torque_comparison(z_opt, p, config_name)
% plot_torque_comparison.m
% =========================================================================
% Plots optimized torque profiles and compares with Week 1 baseline.
%
% INPUTS:
%   z_opt       – [4M×1] optimized decision variables
%   p           – parameter struct from params.m
%   config_name – string: 'simple', 'moderate', 'hard'
%
% OUTPUT:
%   Figure with 4 subplots showing optimized vs baseline torques
% =========================================================================

%% ── Setup ─────────────────────────────────────────────────────────────────
M = p.opt.M;
N = p.N;
t_release = p.sim.t_release;

% Week 1 baseline parameters
A_baseline = [15.0; 8.0; 4.0; 0.15];
omega_baseline = [0.5; 0.30; 0.35; 0.4];
phi_baseline = [0; -0.3; -0.6; 0];

% Time vectors
dt_interval = t_release / M;
t_edges = linspace(0, t_release, M+1);
t_fine = linspace(0, t_release, 500);  % Fine grid for baseline

% Reshape optimized torques
z_mat = reshape(z_opt, [N, M]);

%% ── Create figure ─────────────────────────────────────────────────────────
fig = figure('Name', sprintf('Torque Comparison - %s', upper(config_name)), ...
    'Position', [100, 100, 1400, 900], 'Color', 'w');

joint_names = {'Joint 1 (Shoulder)', 'Joint 2', 'Joint 3', 'Joint 4 (Gripper)'};
colors = lines(2);  % [optimized; baseline]

%% ── Plot each joint ───────────────────────────────────────────────────────
for i = 1:N
    subplot(2, 2, i);
    hold on;
    grid on;
    
    % Baseline sinusoidal torque
    tau_baseline = A_baseline(i) * sin(omega_baseline(i)*t_fine + phi_baseline(i));
    plot(t_fine, tau_baseline, '--', 'Color', colors(2,:), 'LineWidth', 2.0, ...
        'DisplayName', 'Week 1 Baseline');
    
    % Optimized piecewise-constant torque
    stairs(t_edges, [z_mat(i,:), z_mat(i,end)], '-', ...
        'Color', colors(1,:), 'LineWidth', 2.0, ...
        'DisplayName', 'Week 2 Optimized');
    
    % Torque limits
    yline(p.lim.tau_max(i), 'r--', 'LineWidth', 1.0, 'DisplayName', 'Limits');
    yline(p.lim.tau_min(i), 'r--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    
    xlabel('Time [s]');
    ylabel('Torque [N·m]');
    title(joint_names{i});
    legend('Location', 'best');
    
    % Y-axis limits (symmetric around 0 for better visualization)
    ylim_val = max(abs([p.lim.tau_min(i), p.lim.tau_max(i)])) * 1.1;
    ylim([-ylim_val, ylim_val]);
end

sgtitle(sprintf('Optimized vs Baseline Torque Profiles (%s)', upper(config_name)), ...
    'FontSize', 14, 'FontWeight', 'bold');

drawnow;

end
