%GOLF_SWING_DYNAMICS Core dynamics functions for golf swing simulation
%
%   This file contains the mathematical model of the golf swing as a
%   two-link mechanical system. The model uses Lagrangian mechanics
%   to derive the equations of motion for the arm-club system.
%
%   Physical Model:
%       - Link 1 (Arm): Rotates about shoulder joint at origin
%       - Link 2 (Club): Rotates about wrist joint at end of arm
%       - State vector: q = [theta; theta_dot; psi; psi_dot]
%         where theta = arm angle, psi = club angle
%
%   Functions:
%       golf_swing_ode      - ODE system right-hand side
%       golf_swing_mass     - Mass matrix for DAE formulation
%       compute_wrist_torque - Wrist torque model
%       run_simulation      - Main simulation driver
%       compute_impact      - Compute swing trajectory and impact metrics
%
%   Author: Golf Swing MBD Project
%   Date: 2024

function [t, q, values_swing, values_impact] = run_simulation(cfg)
%RUN_SIMULATION Execute golf swing ODE simulation
%
%   Solves the golf swing equations of motion using MATLAB's ODE45 solver
%   with a mass matrix formulation.
%
%   Input:
%       cfg - Configuration structure from config_golf_swing()
%
%   Outputs:
%       t            - Time vector [s]
%       q            - State matrix [theta, theta_dot, psi, psi_dot]
%       values_swing - Structure with swing trajectory data
%       values_impact - Structure with impact metrics

    % Build parameter structure for ODE functions
    params = build_params_struct(cfg);

    % Create time vector
    tspan = linspace(0, cfg.solver.t_final, cfg.solver.resolution);

    % Initial conditions vector
    y0 = [cfg.initial.theta0; cfg.initial.theta_dot0; ...
          cfg.initial.psi0; cfg.initial.psi_dot0];

    % Configure ODE solver with mass matrix
    opts = odeset('Mass', @(t, q) golf_swing_mass(t, q, params));

    % Solve ODE system
    [t, q] = ode45(@(t, q) golf_swing_ode(t, q, params), tspan, y0, opts);

    % Compute swing trajectory and impact metrics
    [values_swing, values_impact] = compute_impact(cfg, t, q);

end

function params = build_params_struct(cfg)
%BUILD_PARAMS_STRUCT Create parameter structure for ODE functions
%
%   Extracts and organizes parameters needed by the ODE system.

    params.IA = cfg.derived.IA;
    params.IC = cfg.derived.IC;
    params.MLS = cfg.derived.MLS;
    params.TH = cfg.torque.shoulder;
    params.k = cfg.derived.k;
    params.k0 = cfg.derived.k0;
    params.k1 = cfg.derived.k1;
    params.torque_multiplier = cfg.torque.wrist_multiplier;
    params.torque_offset = cfg.torque.wrist_offset;

end

function dydt = golf_swing_ode(t, q, params)
%GOLF_SWING_ODE Right-hand side of golf swing ODE system
%
%   Implements the equations of motion for the two-link golf swing model.
%   Uses Lagrangian mechanics with the following state vector:
%       q = [theta; theta_dot; psi; psi_dot]
%
%   The equations of motion are:
%       d(theta)/dt = theta_dot
%       IA*d(theta_dot)/dt + MLS*cos(psi-theta)*d(psi_dot)/dt =
%           MLS*psi_dot^2*sin(psi-theta) + TH - TW
%       d(psi)/dt = psi_dot
%       MLS*cos(psi-theta)*d(theta_dot)/dt + IC*d(psi_dot)/dt =
%           -MLS*theta_dot^2*sin(psi-theta) + TW
%
%   Inputs:
%       t      - Current time [s]
%       q      - State vector [theta; theta_dot; psi; psi_dot]
%       params - Parameter structure
%
%   Output:
%       dydt - State derivative vector

    % Extract parameters
    MLS = params.MLS;
    TH = params.TH;

    % Compute wrist torque
    TW = compute_wrist_torque(t, params);

    % State derivatives
    theta_dot = q(2);
    psi_dot = q(4);
    alpha = q(3) - q(1);  % Relative angle between club and arm

    % Right-hand side of equations of motion
    % Note: Mass matrix handles the inertia terms on LHS
    dydt = [theta_dot;                                    % d(theta)/dt
            MLS * psi_dot^2 * sin(alpha) + TH - TW;       % Torque equation for arm
            psi_dot;                                       % d(psi)/dt
            -MLS * theta_dot^2 * sin(alpha) + TW];        % Torque equation for club

end

function M = golf_swing_mass(~, q, params)
%GOLF_SWING_MASS Mass matrix for golf swing DAE system
%
%   Returns the mass matrix M for the system M*dq/dt = f(t,q).
%   The mass matrix captures the inertial coupling between the arm and club.
%
%   Mass matrix structure:
%       [1    0    0    0   ] [theta_dot    ]
%       [0    IA   0    M12 ] [theta_ddot   ]
%       [0    0    1    0   ] [psi_dot      ]
%       [0    M12  0    IC  ] [psi_ddot     ]
%
%   where M12 = MLS*cos(psi - theta) is the inertial coupling term.
%
%   Inputs:
%       ~      - Time (unused)
%       q      - State vector [theta; theta_dot; psi; psi_dot]
%       params - Parameter structure
%
%   Output:
%       M - 4x4 mass matrix

    % Extract parameters
    IA = params.IA;
    IC = params.IC;
    MLS = params.MLS;

    % Inertial coupling term
    coupling = MLS * cos(q(3) - q(1));

    % Build mass matrix
    M = zeros(4, 4);
    M(1, 1) = 1;         % Identity for theta_dot equation
    M(2, 2) = IA;        % Arm inertia
    M(2, 4) = coupling;  % Coupling: arm to club
    M(3, 3) = 1;         % Identity for psi_dot equation
    M(4, 2) = coupling;  % Coupling: club to arm
    M(4, 4) = IC;        % Club inertia

end

function TW = compute_wrist_torque(t, params)
%COMPUTE_WRIST_TORQUE Compute wrist torque using biomechanical model
%
%   Implements a torque model based on the "delayed release" concept where
%   the wrist torque is initially held to create wrist cock, then released
%   during the downswing.
%
%   The model uses a Heaviside step function to switch the torque at the
%   optimal release time (k0/k1).
%
%   Inputs:
%       t      - Current time [s]
%       params - Parameter structure containing:
%                  TH - shoulder torque
%                  k, k0, k1 - system inertia parameters
%                  torque_multiplier, torque_offset - scaling factors
%
%   Output:
%       TW - Wrist torque [N*m]

    % Extract parameters
    TH = params.TH;
    k = params.k;
    k0 = params.k0;
    k1 = params.k1;
    multiplier = params.torque_multiplier;
    offset = params.torque_offset;

    % Release time
    t_release = k0 / k1;

    % Base wrist torque (before release)
    TW_base = -((1 - heaviside(t - t_release)) .* ((TH / k) .* (k0 - k1 * t)));

    % Apply scaling and offset
    TW = offset + multiplier * TW_base;

end

function [values_swing, values_impact] = compute_impact(cfg, t, q)
%COMPUTE_IMPACT Compute swing trajectory and detect ball impact
%
%   Analyzes the swing trajectory to find when the club head reaches
%   the ball position. Computes club head velocity and angle at impact.
%
%   Inputs:
%       cfg - Configuration structure
%       t   - Time vector [s]
%       q   - State matrix [theta, theta_dot, psi, psi_dot]
%
%   Outputs:
%       values_swing  - Structure with full swing trajectory data
%       values_impact - Structure with impact metrics:
%                         tend_idx   - Index of impact time
%                         time       - Impact time [s]
%                         clubspeed  - Club head speed at impact [m/s]
%                         clubangle  - Club angle at impact [deg]

    % Initialize output structures
    values_swing = struct();
    values_impact = struct();

    % Get dimensions
    n = length(t);
    drel = nan(n, 1);
    clubspeed = nan(n, 1);

    % Extract geometry parameters
    La = cfg.arm.length;
    Lc = cfg.club.length;
    lc = cfg.club.effective_length;
    ball_x = cfg.ball.x;
    ball_y = cfg.ball.y;

    % Scan trajectory for impact
    for j = 1:n
        % Current state
        theta = q(j, 1);
        theta_dot = q(j, 2);
        psi = q(j, 3);
        psi_dot = q(j, 4);

        % Club head position
        x_clubhead = La * cos(theta) + lc * cos(psi);
        y_clubhead = La * sin(theta) + lc * sin(psi);

        % Distance to ball (squared)
        drel(j) = (x_clubhead - ball_x)^2 + (y_clubhead - ball_y)^2;

        % Club head velocity magnitude (using velocity composition)
        clubspeed_sq = (La * theta_dot)^2 + (Lc * psi_dot)^2 + ...
                       2 * La * Lc * cos(psi - theta) * psi_dot * theta_dot;
        clubspeed(j) = sqrt(clubspeed_sq);

        % Check for impact (club passes ball x-position)
        if x_clubhead > ball_x
            disp("Downswing completed");

            tend_idx = j - 1;

            % Impact metrics
            values_impact.tend_idx = tend_idx;
            values_impact.time = t(tend_idx);
            values_impact.clubspeed = sqrt(clubspeed_sq);
            values_impact.clubangle = rad2deg(q(tend_idx, 3));

            % Swing trajectory (up to impact)
            values_swing.time = t(1:tend_idx);
            values_swing.theta = rad2deg(q(1:tend_idx, 1));
            values_swing.psi = rad2deg(q(1:tend_idx, 3));
            values_swing.clubspeed = clubspeed(1:tend_idx);
            values_swing.clubangle = rad2deg(q(1:tend_idx, 3));
            values_swing.drel = drel(1:tend_idx);

            return;
        end
    end

    % No impact detected - return full trajectory
    warning('No impact detected - club did not reach ball position');
    values_impact.tend_idx = n;
    values_impact.time = t(end);
    values_impact.clubspeed = clubspeed(end);
    values_impact.clubangle = rad2deg(q(end, 3));

    values_swing.time = t;
    values_swing.theta = rad2deg(q(:, 1));
    values_swing.psi = rad2deg(q(:, 3));
    values_swing.clubspeed = clubspeed;
    values_swing.clubangle = rad2deg(q(:, 3));
    values_swing.drel = drel;

end

function cfg = update_config_mass(cfg, new_mass)
%UPDATE_CONFIG_MASS Update configuration with new club mass
%
%   Updates the club mass and recomputes all derived parameters.
%   This is used for parametric studies varying club mass.
%
%   Inputs:
%       cfg      - Current configuration structure
%       new_mass - New club mass [kg]
%
%   Output:
%       cfg - Updated configuration structure

    cfg.club.mass = new_mass;

    % Recompute derived parameters
    mc = cfg.club.mass;
    La = cfg.arm.length;
    lc = cfg.club.effective_length;
    Ia = cfg.arm.inertia;
    Ic = cfg.club.inertia;
    TH = cfg.torque.shoulder;
    alpha0 = cfg.initial.alpha0;

    cfg.derived.IA = Ia + mc * La^2;
    cfg.derived.IC = Ic + mc * lc^2;
    cfg.derived.MLS = mc * La * lc;
    cfg.derived.k = cfg.derived.IA + cfg.derived.IC + 2 * cfg.derived.MLS * cos(alpha0);
    cfg.derived.k0 = 0.5 * (cfg.derived.IC - cfg.derived.IA);
    cfg.derived.k1 = (TH / cfg.derived.k) * cfg.derived.MLS * sin(alpha0);

end
