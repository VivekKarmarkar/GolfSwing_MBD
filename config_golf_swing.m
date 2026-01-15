function cfg = config_golf_swing()
%CONFIG_GOLF_SWING Returns configuration structure for golf swing simulation
%
%   This function centralizes all physical parameters, initial conditions,
%   solver settings, and visualization options for the golf swing MBD model.
%
%   Usage:
%       cfg = config_golf_swing();
%       cfg.club.mass = 0.35;  % Override specific parameter
%
%   Returns:
%       cfg - Structure containing all simulation parameters
%
%   Author: Golf Swing MBD Project
%   Date: 2024

    %% Physical Constants - Golfer Body
    cfg.arm.length = 0.615;              % La: Arm length [m]
    cfg.arm.inertia = 1.15;              % Ia: Arm moment of inertia [kg*m^2]

    %% Physical Constants - Golf Club
    cfg.club.mass = 0.394;               % mc: Club mass [kg]
    cfg.club.length = 1.105;             % Lc: Club total length [m]
    cfg.club.effective_length = 0.753;   % lc: Effective club length to COM [m]
    cfg.club.inertia = 0.077;            % Ic: Club moment of inertia [kg*m^2]

    %% Torque Model Parameters
    cfg.torque.shoulder = 250;           % TH: Shoulder/hub torque [N*m]
    cfg.torque.wrist_multiplier = 5;     % Wrist torque scaling factor
    cfg.torque.wrist_offset = 20;        % Wrist torque offset [N*m]

    %% Initial Conditions
    cfg.initial.alpha0 = deg2rad(-124);  % Initial angle between arm and club [rad]
    cfg.initial.theta0 = deg2rad(-166);  % Initial arm angle [rad]
    cfg.initial.theta_dot0 = 0;          % Initial arm angular velocity [rad/s]
    cfg.initial.psi0 = [];               % Calculated from theta0 + alpha0
    cfg.initial.psi_dot0 = 0;            % Initial club angular velocity [rad/s]

    % Calculate initial club angle
    cfg.initial.psi0 = cfg.initial.theta0 + cfg.initial.alpha0;

    %% Ball Position
    cfg.ball.x = 0.19;                   % Ball x-position [m]
    cfg.ball.y = -1.35;                  % Ball y-position [m]

    %% Solver Settings
    cfg.solver.t_final = 300e-3;         % Simulation end time [s] (300 ms)
    cfg.solver.resolution = 1000;        % Number of time points
    cfg.solver.method = 'ode45';         % ODE solver method

    %% Parametric Study Settings
    cfg.parametric.mass_decrement = -0.005;  % Club mass step [kg]
    cfg.parametric.min_clubspeed = 35;       % Minimum realistic club speed [m/s]
    cfg.parametric.min_clubangle = -90;      % Minimum realistic club angle [deg]
    cfg.parametric.max_clubangle = -80;      % Maximum realistic club angle [deg]

    %% Visualization Settings
    cfg.visualization.swing_resolution = 40;      % Frames between swing snapshots
    cfg.visualization.parametric_resolution = 0.07; % Fraction of swing to show
    cfg.visualization.save_figures = true;        % Save plots to PNG files
    cfg.visualization.figure_format = 'png';      % Output format

    %% Visualization Flags (control which plots to generate)
    cfg.flags.plot_basic = true;              % Basic swing trajectory
    cfg.flags.plot_animation = false;         % Animated video
    cfg.flags.plot_parametric_overlay = false; % Overlaid parametric curves
    cfg.flags.plot_parametric_analysis = true; % Parametric analysis plots
    cfg.flags.plot_parametric_swings = true;  % Individual swing trajectories
    cfg.flags.plot_comparative = true;        % Heavy vs light comparison

    %% Derived Parameters (computed from base parameters)
    cfg = compute_derived_parameters(cfg);

end

function cfg = compute_derived_parameters(cfg)
%COMPUTE_DERIVED_PARAMETERS Calculate derived physical parameters
%
%   Computes composite inertias and coupling terms from base parameters.

    % Extract base parameters
    mc = cfg.club.mass;
    La = cfg.arm.length;
    lc = cfg.club.effective_length;
    Ia = cfg.arm.inertia;
    Ic = cfg.club.inertia;
    TH = cfg.torque.shoulder;
    alpha0 = cfg.initial.alpha0;

    % Composite moment of inertia for arm with club mass
    cfg.derived.IA = Ia + mc * La^2;

    % Composite moment of inertia for club about wrist
    cfg.derived.IC = Ic + mc * lc^2;

    % Mass-length-length coupling term
    cfg.derived.MLS = mc * La * lc;

    % System effective inertia at initial configuration
    cfg.derived.k = cfg.derived.IA + cfg.derived.IC + 2 * cfg.derived.MLS * cos(alpha0);

    % Torque model parameters
    cfg.derived.k0 = 0.5 * (cfg.derived.IC - cfg.derived.IA);
    cfg.derived.k1 = (TH / cfg.derived.k) * cfg.derived.MLS * sin(alpha0);

end

function cfg = update_club_mass(cfg, new_mass)
%UPDATE_CLUB_MASS Update configuration with new club mass
%
%   Updates the club mass and recomputes all derived parameters.
%
%   Inputs:
%       cfg      - Current configuration structure
%       new_mass - New club mass [kg]
%
%   Returns:
%       cfg - Updated configuration structure

    cfg.club.mass = new_mass;
    cfg = compute_derived_parameters(cfg);

end
