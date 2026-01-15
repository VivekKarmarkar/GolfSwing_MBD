%GOLF_SWING_SIMULATION Main simulation driver for golf swing MBD model
%
%   This is the main entry point for the golf swing multibody dynamics
%   simulation. It performs parametric analysis by varying club mass
%   and generates visualization of the results.
%
%   Usage:
%       golf_swing_simulation          % Run with default settings
%       golf_swing_simulation(cfg)     % Run with custom configuration
%
%   The simulation:
%       1. Loads configuration parameters
%       2. Runs parametric study varying club mass
%       3. Filters for "realistic" swings (proper impact angle and speed)
%       4. Generates visualization plots
%
%   Dependencies:
%       config_golf_swing.m     - Configuration parameters
%       golf_swing_dynamics.m   - Physics and ODE functions
%       golf_swing_plotting.m   - Visualization functions
%
%   Author: Golf Swing MBD Project
%   Date: 2024

function swing_data = golf_swing_simulation(cfg)
%GOLF_SWING_SIMULATION Main simulation function
%
%   Runs the golf swing simulation with parametric mass variation.
%
%   Input:
%       cfg - Configuration structure (optional, uses defaults if not provided)
%
%   Output:
%       swing_data - Structure array containing all valid swing results

    %% Initialize Configuration
    if nargin < 1
        cfg = config_golf_swing();
    end

    fprintf('=== Golf Swing MBD Simulation ===\n\n');
    fprintf('Physical Parameters:\n');
    fprintf('  Arm length:    %.3f m\n', cfg.arm.length);
    fprintf('  Club length:   %.3f m\n', cfg.club.length);
    fprintf('  Initial mass:  %.3f kg\n', cfg.club.mass);
    fprintf('  Shoulder torque: %.1f N*m\n', cfg.torque.shoulder);
    fprintf('\n');

    %% Run Parametric Study
    fprintf('Running parametric study...\n');

    % Initialize storage
    swing_data = struct([]);
    parameter_range = [];
    shot_idx = 0;

    % Current club mass
    mc = cfg.club.mass;
    dm = cfg.parametric.mass_decrement;

    % Iterate until swings become unrealistic
    realistic_shot = true;
    max_iterations = 100;  % Safety limit
    iteration = 0;

    while realistic_shot && iteration < max_iterations
        iteration = iteration + 1;

        % Update configuration with current mass
        cfg = update_config_mass(cfg, mc);

        % Run simulation
        [~, ~, values_swing, values_impact] = run_simulation(cfg);

        % Check if swing is realistic
        angle_ok = values_impact.clubangle > cfg.parametric.min_clubangle && ...
                   values_impact.clubangle < cfg.parametric.max_clubangle;
        speed_ok = values_impact.clubspeed > cfg.parametric.min_clubspeed;

        if angle_ok && speed_ok
            shot_idx = shot_idx + 1;

            % Store swing data
            swing_data(shot_idx).mc = mc;
            swing_data(shot_idx).time = values_swing.time;
            swing_data(shot_idx).theta = values_swing.theta;
            swing_data(shot_idx).psi = values_swing.psi;
            swing_data(shot_idx).clubspeed = values_swing.clubspeed;
            swing_data(shot_idx).clubangle = values_swing.clubangle;
            swing_data(shot_idx).drel = values_swing.drel;
            swing_data(shot_idx).values_impact = values_impact;
            swing_data(shot_idx).constants = struct('La', cfg.arm.length, ...
                                                    'Lc', cfg.club.length, ...
                                                    'lc', cfg.club.effective_length);
            swing_data(shot_idx).params = build_params_struct(cfg);

            parameter_range = [parameter_range, mc];

            fprintf('  Shot %d: mc = %.3f kg, v = %.1f m/s, angle = %.1f deg\n', ...
                    shot_idx, mc, values_impact.clubspeed, values_impact.clubangle);

            % Decrease mass for next iteration
            mc = mc + dm;
        else
            realistic_shot = false;
            fprintf('  Stopping: swing with mc = %.3f kg is unrealistic\n', mc);
        end
    end

    numshots = shot_idx;
    fprintf('\nCompleted: %d realistic swings found\n\n', numshots);

    %% Generate Visualizations
    if numshots > 0
        fprintf('Generating plots...\n');
        generate_all_plots(cfg, swing_data, parameter_range);
        fprintf('Plots saved.\n\n');
    else
        warning('No realistic swings found. Check parameters.');
    end

    %% Print Summary
    if numshots > 0
        fprintf('=== Results Summary ===\n');
        fprintf('Mass range: %.3f to %.3f kg\n', parameter_range(1), parameter_range(end));
        fprintf('Speed range: %.1f to %.1f m/s\n', ...
                swing_data(1).values_impact.clubspeed, ...
                swing_data(end).values_impact.clubspeed);
        fprintf('Time range: %.1f to %.1f ms\n', ...
                swing_data(1).values_impact.time * 1000, ...
                swing_data(end).values_impact.time * 1000);
        fprintf('\n');
    end

end

function params = build_params_struct(cfg)
%BUILD_PARAMS_STRUCT Create parameter structure for storage
    params.IA = cfg.derived.IA;
    params.IC = cfg.derived.IC;
    params.MLS = cfg.derived.MLS;
    params.TH = cfg.torque.shoulder;
    params.k = cfg.derived.k;
    params.k0 = cfg.derived.k0;
    params.k1 = cfg.derived.k1;
    params.tf = cfg.solver.t_final;
end

%% Run simulation if called directly
% Uncomment the following line to run automatically when script is executed:
% golf_swing_simulation();
