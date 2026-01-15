%GOLF_SWING_PLOTTING Visualization functions for golf swing simulation
%
%   This module contains all plotting and animation functions for
%   visualizing golf swing simulation results.
%
%   Functions:
%       plot_swing_trajectory      - 2D trajectory of arm and club
%       plot_time_domain           - Angles and speed vs time
%       plot_impact_analysis       - Metrics vs distance to ball
%       plot_torque_model          - Shoulder and wrist torques
%       plot_parametric_overlay    - Multiple swings overlaid
%       plot_parametric_analysis   - Impact metrics vs club mass
%       plot_parametric_swings     - Tiled swing trajectories
%       plot_comparative_swings    - Heavy vs light club comparison
%       create_swing_animation     - Animated video of swing
%
%   Author: Golf Swing MBD Project
%   Date: 2024

function plot_swing_trajectory(cfg, swing_data, shot_idx)
%PLOT_SWING_TRAJECTORY Plot 2D trajectory of golf swing
%
%   Creates a static plot showing the arm and club positions at regular
%   intervals during the downswing.
%
%   Inputs:
%       cfg        - Configuration structure
%       swing_data - Array of swing data structures
%       shot_idx   - Index of shot to plot

    La = cfg.arm.length;
    lc = cfg.club.effective_length;
    resolution = cfg.visualization.swing_resolution;

    data = swing_data(shot_idx);
    values_theta = deg2rad(data.theta);
    values_psi = deg2rad(data.psi);
    tend_idx = data.values_impact.tend_idx;

    figure('WindowState', 'maximized');
    title('Golf Swing Motion - Solved by ODE45');
    hold on;

    for j = 1:tend_idx
        theta = values_theta(j);
        psi = values_psi(j);

        % Arm segment
        xvals = [0, La * cos(theta)];
        yvals = [0, La * sin(theta)];

        % Club segment
        xvals2 = [La * cos(theta), La * cos(theta) + lc * cos(psi)];
        yvals2 = [La * sin(theta), La * sin(theta) + lc * sin(psi)];

        if mod(j, resolution) == 0
            plot(xvals, yvals, 'k', 'HandleVisibility', 'off');
            plot(xvals(1), yvals(1), 'mo', xvals(2), yvals(2), 'ro', ...
                 'HandleVisibility', 'off');
            plot(xvals2, yvals2, 'k', 'HandleVisibility', 'off');
            plot(xvals2(1), yvals2(1), 'ro', xvals2(2), yvals2(2), 'bo', ...
                 'HandleVisibility', 'off');
        end
    end

    % Legend entries
    plot(nan, 'mo', 'DisplayName', 'Shoulder joint');
    plot(nan, 'ro', 'DisplayName', 'Wrist joint');
    plot(nan, 'bo', 'DisplayName', 'Club head');
    plot(cfg.ball.x, cfg.ball.y, '.b', 'MarkerSize', 35, 'DisplayName', 'Ball');

    xlabel('x (m)');
    ylabel('y (m)');
    axis equal;
    grid on;
    legend('Location', 'best');
    hold off;

    if cfg.visualization.save_figures
        saveas(gcf, 'GolfSwingPlot.png');
    end
    close(gcf);

end

function plot_time_domain(cfg, swing_data, shot_idx)
%PLOT_TIME_DOMAIN Plot angles and club speed vs time
%
%   Creates subplots showing arm angle, club angle, and club speed
%   as functions of time during the downswing.

    data = swing_data(shot_idx);
    t = data.time;
    theta = data.theta;
    psi = data.psi;
    clubspeed = data.clubspeed;

    figure('WindowState', 'maximized');
    sgtitle('Time-Domain Analysis');

    subplot(3, 1, 1);
    plot(t, theta, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\theta (degrees)');
    title('Arm Angle \theta');
    grid on;

    subplot(3, 1, 2);
    plot(t, psi, 'r', 'LineWidth', 1.5);
    hold on;
    yline(-90, 'k--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('\psi (degrees)');
    title('Club Angle \psi');
    grid on;

    subplot(3, 1, 3);
    plot(t, clubspeed, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Club Speed (m/s)');
    title('Club Head Speed');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'TimeDomainPlot.png');
    end
    close(gcf);

end

function plot_impact_analysis(cfg, swing_data, shot_idx)
%PLOT_IMPACT_ANALYSIS Plot metrics vs distance to ball
%
%   Shows club angle and speed as functions of distance to the ball,
%   useful for understanding the approach to impact.

    data = swing_data(shot_idx);
    drel = data.drel;
    psi = data.psi;
    clubspeed = data.clubspeed;

    figure('WindowState', 'maximized');
    sgtitle('Impact Analysis');

    subplot(2, 1, 1);
    plot(drel, psi, 'r', 'LineWidth', 1.5);
    set(gca, 'XDir', 'reverse');
    hold on;
    yline(-90, 'k--', 'LineWidth', 1);
    xlabel('Distance to Ball (m^2)');
    ylabel('\psi (degrees)');
    title('Club Angle vs Distance to Ball');
    grid on;

    subplot(2, 1, 2);
    plot(drel, clubspeed, 'r', 'LineWidth', 1.5);
    set(gca, 'XDir', 'reverse');
    xlabel('Distance to Ball (m^2)');
    ylabel('Club Speed (m/s)');
    title('Club Speed vs Distance to Ball');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'ImpactPlot.png');
    end
    close(gcf);

end

function plot_torque_model(cfg, swing_data, shot_idx)
%PLOT_TORQUE_MODEL Plot shoulder and wrist torques vs time
%
%   Visualizes the torque model used in the simulation, showing
%   the constant shoulder torque and the switching wrist torque.

    data = swing_data(shot_idx);
    params = data.params;
    t = data.time;

    TH = params.TH;
    k = params.k;
    k0 = params.k0;
    k1 = params.k1;
    multiplier = cfg.torque.wrist_multiplier;
    offset = cfg.torque.wrist_offset;

    % Compute torque profiles
    t_release = k0 / k1;
    TW = -((1 - heaviside(t - t_release)) .* ((TH / k) .* (k0 - k1 * t)));
    TW = offset + multiplier * TW;
    TH_profile = TH * ones(size(t));

    figure('WindowState', 'maximized');
    sgtitle('Torque Model');

    subplot(2, 1, 1);
    plot(t, TH_profile, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('T_H (N\cdotm)');
    title('Shoulder (Hub) Torque');
    grid on;

    subplot(2, 1, 2);
    plot(t, TW, 'r', 'LineWidth', 1.5);
    hold on;
    xline(t_release, 'k--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('T_W (N\cdotm)');
    title('Wrist Torque');
    legend('T_W', 'Release time', 'Location', 'best');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'TorquePlot.png');
    end
    close(gcf);

end

function plot_parametric_analysis(cfg, swing_data, parameter_range)
%PLOT_PARAMETRIC_ANALYSIS Plot impact metrics vs club mass
%
%   Creates subplots showing how key impact metrics vary with
%   club mass across the parametric study.

    numshots = length(swing_data);

    % Extract metrics
    ti_list = nan(numshots, 1);
    cs_avg_list = nan(numshots, 1);
    csi_list = nan(numshots, 1);
    cai_list = nan(numshots, 1);

    for j = 1:numshots
        ti_list(j) = swing_data(j).values_impact.time;
        cs_avg_list(j) = mean(swing_data(j).clubspeed);
        csi_list(j) = swing_data(j).values_impact.clubspeed;
        cai_list(j) = swing_data(j).values_impact.clubangle;
    end

    figure('WindowState', 'maximized');
    sgtitle('Parametric Analysis: Effect of Club Mass');

    subplot(2, 2, 1);
    plot(parameter_range, ti_list, 'b-o', 'LineWidth', 1.5);
    xlabel('Club Mass m_c (kg)');
    ylabel('Downswing Time (s)');
    title('Downswing Duration');
    grid on;

    subplot(2, 2, 2);
    plot(parameter_range, csi_list, 'k-o', 'LineWidth', 1.5);
    xlabel('Club Mass m_c (kg)');
    ylabel('Club Speed (m/s)');
    title('Impact Club Speed');
    grid on;

    subplot(2, 2, 3);
    plot(parameter_range, cai_list, 'r-o', 'LineWidth', 1.5);
    xlabel('Club Mass m_c (kg)');
    ylabel('\psi (degrees)');
    title('Impact Club Angle');
    grid on;

    subplot(2, 2, 4);
    plot(parameter_range, cs_avg_list, 'm-o', 'LineWidth', 1.5);
    xlabel('Club Mass m_c (kg)');
    ylabel('Average Speed (m/s)');
    title('Average Club Speed');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'ParametricAnalysisPlot.png');
    end
    close(gcf);

end

function plot_parametric_swings(cfg, swing_data)
%PLOT_PARAMETRIC_SWINGS Plot tiled swing trajectories for different masses
%
%   Creates a tiled layout showing swing trajectories for different
%   club masses, allowing visual comparison of swing patterns.

    La = cfg.arm.length;
    lc = cfg.club.effective_length;
    numshots = length(swing_data);
    step = max(1, floor(numshots / 4));  % Select ~4 swings to display

    figure('WindowState', 'maximized');
    tiledlayout(2, 2, 'TileSpacing', 'compact');
    sgtitle('Golf Swing Motion: Varying Club Mass');

    plot_count = 0;
    for shot_idx = 1:step:numshots
        if plot_count >= 4
            break;
        end
        plot_count = plot_count + 1;

        data = swing_data(shot_idx);
        values_theta = deg2rad(data.theta);
        values_psi = deg2rad(data.psi);
        tend_idx = data.values_impact.tend_idx;

        resolution = ceil(cfg.visualization.parametric_resolution * tend_idx);

        nexttile;
        title(sprintf('m_c = %.3f kg', data.mc));
        hold on;

        for j = flip(1:tend_idx)
            theta = values_theta(j);
            psi = values_psi(j);

            xvals = [0, La * cos(theta)];
            yvals = [0, La * sin(theta)];
            xvals2 = [La * cos(theta), La * cos(theta) + lc * cos(psi)];
            yvals2 = [La * sin(theta), La * sin(theta) + lc * sin(psi)];

            if mod(tend_idx - j, resolution) == 0
                plot(xvals, yvals, 'k', 'HandleVisibility', 'off');
                plot(xvals(1), yvals(1), 'mo', xvals(2), yvals(2), 'ro', ...
                     'HandleVisibility', 'off');
                plot(xvals2, yvals2, 'k', 'HandleVisibility', 'off');
                plot(xvals2(1), yvals2(1), 'ro', xvals2(2), yvals2(2), 'bo', ...
                     'HandleVisibility', 'off');
            end
        end

        plot(cfg.ball.x, cfg.ball.y, '.b', 'MarkerSize', 25, 'DisplayName', 'Ball');

        if plot_count == 1
            plot(nan, 'mo', 'DisplayName', 'Shoulder');
            plot(nan, 'ro', 'DisplayName', 'Wrist');
            plot(nan, 'bo', 'DisplayName', 'Club head');
            legend('Location', 'best');
        end

        xlabel('x (m)');
        ylabel('y (m)');
        axis equal;
        grid on;
        hold off;
    end

    if cfg.visualization.save_figures
        saveas(gcf, 'ParametricSwingsPlot.png');
    end
    close(gcf);

end

function plot_comparative_swings(cfg, swing_data)
%PLOT_COMPARATIVE_SWINGS Compare heavy vs light club swings
%
%   Overlays the heaviest and lightest club swings on the same plot
%   to visualize the effect of club mass on swing trajectory.

    La = cfg.arm.length;
    lc = cfg.club.effective_length;
    numshots = length(swing_data);

    if numshots < 2
        warning('Need at least 2 shots for comparative plot');
        return;
    end

    figure('WindowState', 'maximized');
    title('Golf Swing Comparison: Heavy vs Light Club');
    hold on;

    shot_indices = [1, numshots];  % First (heavy) and last (light)
    line_widths = [2.5, 1.5];
    colors = {'k', 'm'};
    names = {'Heavy club', 'Light club'};

    for s = 1:2
        shot_idx = shot_indices(s);
        data = swing_data(shot_idx);

        values_theta = deg2rad(data.theta);
        values_psi = deg2rad(data.psi);
        tend_idx = data.values_impact.tend_idx;

        resolution = ceil(cfg.visualization.parametric_resolution * tend_idx);

        for j = flip(1:tend_idx)
            theta = values_theta(j);
            psi = values_psi(j);

            xvals = [0, La * cos(theta)];
            yvals = [0, La * sin(theta)];
            xvals2 = [La * cos(theta), La * cos(theta) + lc * cos(psi)];
            yvals2 = [La * sin(theta), La * sin(theta) + lc * sin(psi)];

            if mod(tend_idx - j, resolution) == 0
                plot(xvals, yvals, colors{s}, 'LineWidth', line_widths(s), ...
                     'HandleVisibility', 'off');
                plot(xvals(1), yvals(1), 'mo', xvals(2), yvals(2), 'ro', ...
                     'HandleVisibility', 'off');
                plot(xvals2, yvals2, colors{s}, 'LineWidth', line_widths(s), ...
                     'HandleVisibility', 'off');
                plot(xvals2(1), yvals2(1), 'ro', xvals2(2), yvals2(2), 'bo', ...
                     'HandleVisibility', 'off');
            end

            if j == tend_idx
                plot(nan, colors{s}, 'LineWidth', line_widths(s), ...
                     'DisplayName', sprintf('%s (m_c = %.3f kg)', names{s}, data.mc));
            end
        end
    end

    plot(nan, 'mo', 'DisplayName', 'Shoulder joint');
    plot(nan, 'ro', 'DisplayName', 'Wrist joint');
    plot(nan, 'bo', 'DisplayName', 'Club head');
    plot(cfg.ball.x, cfg.ball.y, '.b', 'MarkerSize', 35, 'DisplayName', 'Ball');

    xlabel('x (m)');
    ylabel('y (m)');
    axis equal;
    grid on;
    legend('Location', 'best');
    hold off;

    if cfg.visualization.save_figures
        saveas(gcf, 'ComparativeSwingsPlot.png');
    end
    close(gcf);

end

function plot_parametric_overlay(cfg, swing_data)
%PLOT_PARAMETRIC_OVERLAY Overlay multiple swings on same plots
%
%   Creates time-domain and impact analysis plots with multiple
%   club masses overlaid for comparison.

    numshots = length(swing_data);
    step = max(1, floor(numshots / 4));

    % Time-domain overlay
    figure('WindowState', 'maximized');
    sgtitle('Time-Domain Analysis: Parametric Overlay');

    for shot_idx = 1:step:numshots
        data = swing_data(shot_idx);
        t_norm = data.time / data.values_impact.time;
        mc = data.mc;
        label = sprintf('m_c = %.3f', mc);

        subplot(3, 1, 1);
        plot(t_norm, data.theta, 'LineWidth', 1.5, 'DisplayName', label);
        hold on;

        subplot(3, 1, 2);
        plot(t_norm, data.psi, 'LineWidth', 1.5, 'DisplayName', label);
        hold on;

        subplot(3, 1, 3);
        plot(t_norm, data.clubspeed, 'LineWidth', 1.5, 'DisplayName', label);
        hold on;
    end

    subplot(3, 1, 1);
    xlabel('Normalized Time');
    ylabel('\theta (degrees)');
    title('Arm Angle');
    legend('Location', 'northwest');
    grid on;

    subplot(3, 1, 2);
    yline(-90, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Normalized Time');
    ylabel('\psi (degrees)');
    title('Club Angle');
    legend('Location', 'northwest');
    grid on;

    subplot(3, 1, 3);
    xlabel('Normalized Time');
    ylabel('Club Speed (m/s)');
    title('Club Head Speed');
    legend('Location', 'northwest');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'TimeDomainParametricPlot.png');
    end
    close(gcf);

    % Impact analysis overlay
    figure('WindowState', 'maximized');
    sgtitle('Impact Analysis: Parametric Overlay');

    for shot_idx = 1:step:numshots
        data = swing_data(shot_idx);
        mc = data.mc;
        label = sprintf('m_c = %.3f', mc);

        subplot(2, 1, 1);
        plot(data.drel, data.psi, 'LineWidth', 1.5, 'DisplayName', label);
        hold on;

        subplot(2, 1, 2);
        plot(data.drel, data.clubspeed, 'LineWidth', 1.5, 'DisplayName', label);
        hold on;
    end

    subplot(2, 1, 1);
    set(gca, 'XDir', 'reverse');
    yline(-90, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    xlabel('Distance to Ball (m^2)');
    ylabel('\psi (degrees)');
    title('Club Angle vs Distance');
    legend('Location', 'northwest');
    grid on;

    subplot(2, 1, 2);
    set(gca, 'XDir', 'reverse');
    xlabel('Distance to Ball (m^2)');
    ylabel('Club Speed (m/s)');
    title('Club Speed vs Distance');
    legend('Location', 'northwest');
    grid on;

    if cfg.visualization.save_figures
        saveas(gcf, 'ImpactParametricPlot.png');
    end
    close(gcf);

end

function create_swing_animation(cfg, swing_data, shot_idx, filename)
%CREATE_SWING_ANIMATION Create animated video of golf swing
%
%   Generates an MP4 video showing the golf swing animation.
%
%   Inputs:
%       cfg        - Configuration structure
%       swing_data - Array of swing data structures
%       shot_idx   - Index of shot to animate
%       filename   - Output video filename (optional)

    if nargin < 4
        filename = 'golf_swing_animation.mp4';
    end

    La = cfg.arm.length;
    lc = cfg.club.effective_length;

    data = swing_data(shot_idx);
    values_theta = deg2rad(data.theta);
    values_psi = deg2rad(data.psi);
    tend_idx = data.values_impact.tend_idx;

    % Create video writer
    vidObj = VideoWriter(filename, 'MPEG-4');
    vidObj.FrameRate = 30;
    open(vidObj);

    fig = figure('WindowState', 'maximized');

    for j = 1:tend_idx
        clf;

        % Plot ball
        plot(cfg.ball.x, cfg.ball.y, '.b', 'MarkerSize', 35);
        xlim([-1.6, 0.6]);
        ylim([-1.5, 1]);
        grid on;
        hold on;

        theta = values_theta(j);
        psi = values_psi(j);

        % Arm segment
        xvals = [0, La * cos(theta)];
        yvals = [0, La * sin(theta)];

        % Club segment
        xvals2 = [La * cos(theta), La * cos(theta) + lc * cos(psi)];
        yvals2 = [La * sin(theta), La * sin(theta) + lc * sin(psi)];

        plot(xvals, yvals, 'k', 'LineWidth', 2);
        plot(xvals(1), yvals(1), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
        plot(xvals(2), yvals(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

        plot(xvals2, yvals2, 'k', 'LineWidth', 2);
        plot(xvals2(2), yvals2(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

        xlabel('x (m)');
        ylabel('y (m)');
        title(sprintf('Golf Swing Animation - t = %.3f s', data.time(j)));

        drawnow;
        frame = getframe(fig);
        writeVideo(vidObj, frame);
    end

    close(vidObj);
    close(fig);

    fprintf('Animation saved to: %s\n', filename);

end

function generate_all_plots(cfg, swing_data, parameter_range)
%GENERATE_ALL_PLOTS Generate all enabled plots
%
%   Master function that checks visualization flags and generates
%   all enabled plot types.
%
%   Inputs:
%       cfg             - Configuration structure
%       swing_data      - Array of swing data structures
%       parameter_range - Array of club masses used

    % Basic plots for first shot
    if cfg.flags.plot_basic && ~isempty(swing_data)
        plot_swing_trajectory(cfg, swing_data, 1);
        plot_time_domain(cfg, swing_data, 1);
        plot_impact_analysis(cfg, swing_data, 1);
        plot_torque_model(cfg, swing_data, 1);
    end

    % Animation
    if cfg.flags.plot_animation && ~isempty(swing_data)
        create_swing_animation(cfg, swing_data, 1);
    end

    % Parametric overlay plots
    if cfg.flags.plot_parametric_overlay && length(swing_data) > 1
        plot_parametric_overlay(cfg, swing_data);
    end

    % Parametric analysis
    if cfg.flags.plot_parametric_analysis && length(swing_data) > 1
        plot_parametric_analysis(cfg, swing_data, parameter_range);
    end

    % Parametric swing trajectories
    if cfg.flags.plot_parametric_swings && length(swing_data) > 1
        plot_parametric_swings(cfg, swing_data);
    end

    % Comparative swings
    if cfg.flags.plot_comparative && length(swing_data) > 1
        plot_comparative_swings(cfg, swing_data);
    end

end
