# Golf Swing Multibody Dynamics Simulation

A MATLAB-based biomechanical simulation of the golf swing using multibody dynamics (MBD). This project models the arm-club system as a two-link kinematic chain to analyze how club mass affects swing dynamics, club head speed, and impact characteristics.

## Table of Contents

- [Overview](#overview)
- [Physical Model](#physical-model)
- [Mathematical Formulation](#mathematical-formulation)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Configuration](#configuration)
- [Output](#output)
- [Theory](#theory)
- [References](#references)

## Overview

This simulation solves the equations of motion for a simplified golf swing model using MATLAB's ODE45 solver. The model captures the essential dynamics of the downswing phase, including:

- **Coupled rotation** of arm and club segments
- **Wrist torque release** timing optimization
- **Club head velocity** at ball impact
- **Parametric studies** on club mass effects

The simulation demonstrates that lighter clubs achieve higher club head speeds but with different swing timing characteristics.

## Physical Model

The golf swing is modeled as a **two-link planar mechanism**:

```
                    Shoulder (Origin)
                         o
                        /
                       /  Arm (Link 1)
                      /   Length: La = 0.615 m
                     /    Angle: θ (theta)
                    /
                   o  Wrist Joint
                    \
                     \  Club (Link 2)
                      \ Length: lc = 0.753 m (effective)
                       \ Angle: ψ (psi)
                        \
                         o  Club Head

                         O  Ball
```

### Degrees of Freedom

| Variable | Symbol | Description |
|----------|--------|-------------|
| Arm angle | θ | Rotation of arm about shoulder |
| Arm angular velocity | θ̇ | Rate of change of arm angle |
| Club angle | ψ | Rotation of club about wrist |
| Club angular velocity | ψ̇ | Rate of change of club angle |

### Physical Parameters

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Arm length | La | 0.615 | m |
| Club length | Lc | 1.105 | m |
| Effective club length | lc | 0.753 | m |
| Club mass | mc | 0.394 | kg |
| Arm moment of inertia | Ia | 1.15 | kg·m² |
| Club moment of inertia | Ic | 0.077 | kg·m² |
| Shoulder torque | TH | 250 | N·m |

## Mathematical Formulation

### Equations of Motion

The system dynamics are derived using Lagrangian mechanics. The equations of motion in matrix form:

```
M(q) · q̈ = f(q, q̇, t)
```

Where the **mass matrix** M is:

```
    ┌                              ┐
    │  1     0      0       0      │
M = │  0    IA      0    M_LS·cos α│
    │  0     0      1       0      │
    │  0  M_LS·cos α 0      IC     │
    └                              ┘
```

And α = ψ - θ is the relative angle between club and arm.

### Derived Inertia Terms

- **IA** = Ia + mc·La² (composite arm inertia)
- **IC** = Ic + mc·lc² (composite club inertia)
- **M_LS** = mc·La·lc (mass-length coupling)

### Torque Model

The shoulder applies a constant torque TH. The wrist torque TW follows a "delayed release" model:

```
TW = offset + multiplier × TW_base

where:
TW_base = -(1 - H(t - t_release)) × (TH/k) × (k₀ - k₁·t)
```

H(t) is the Heaviside step function, and t_release = k₀/k₁ is the optimal release time.

### Club Head Velocity

At any instant, the club head velocity magnitude is:

```
v = √[(La·θ̇)² + (Lc·ψ̇)² + 2·La·Lc·cos(α)·θ̇·ψ̇]
```

## Project Structure

```
GolfSwing_MBD/
├── README.md                    # This file
├── golf_swing_simulation.m      # Main entry point
├── config_golf_swing.m          # Configuration parameters
├── golf_swing_dynamics.m        # Physics and ODE functions
├── golf_swing_plotting.m        # Visualization functions
├── GolfSwing_MBD.m             # Legacy: Version 1 (single swing)
├── GolfSwing_MBD_2.m           # Legacy: Version 2 (parametric)
└── GolfSwing_MBD_3.m           # Legacy: Version 3 (comparative)
```

### Module Descriptions

| File | Purpose |
|------|---------|
| `golf_swing_simulation.m` | Main driver script - runs parametric study and generates plots |
| `config_golf_swing.m` | Centralized configuration for all physical parameters and settings |
| `golf_swing_dynamics.m` | Core physics: ODE system, mass matrix, torque model, impact detection |
| `golf_swing_plotting.m` | All visualization functions: trajectories, time-domain, parametric analysis |

## Getting Started

### Requirements

- **MATLAB R2019a** or later
- **Symbolic Math Toolbox** (for `heaviside` function)
- *Optional:* Image Processing Toolbox (for video export)

### Quick Start

1. Clone or download this repository
2. Open MATLAB and navigate to the project directory
3. Run the simulation:

```matlab
>> golf_swing_simulation
```

## Usage

### Basic Simulation

Run with default parameters:

```matlab
% Run full parametric study
swing_data = golf_swing_simulation();
```

### Custom Configuration

Modify parameters before running:

```matlab
% Load default configuration
cfg = config_golf_swing();

% Customize parameters
cfg.club.mass = 0.35;                    % Lighter club
cfg.torque.shoulder = 280;               % Stronger golfer
cfg.flags.plot_animation = true;         % Enable animation

% Run with custom config
swing_data = golf_swing_simulation(cfg);
```

### Running Legacy Scripts

The original scripts are preserved for reference:

```matlab
% Version 1: Single swing analysis
GolfSwing_MBD

% Version 2: Parametric mass study
GolfSwing_MBD_2

% Version 3: Comparative visualization
GolfSwing_MBD_3
```

## Configuration

### Visualization Flags

Control which plots are generated:

```matlab
cfg.flags.plot_basic = true;              % Swing trajectory, time-domain, torque
cfg.flags.plot_animation = false;         % MP4 video animation
cfg.flags.plot_parametric_overlay = false;% Multiple masses overlaid
cfg.flags.plot_parametric_analysis = true;% Metrics vs club mass
cfg.flags.plot_parametric_swings = true;  % Tiled swing trajectories
cfg.flags.plot_comparative = true;        % Heavy vs light comparison
```

### Parametric Study Settings

Configure the mass variation study:

```matlab
cfg.parametric.mass_decrement = -0.005;   % Step size (kg)
cfg.parametric.min_clubspeed = 35;        % Minimum valid speed (m/s)
cfg.parametric.min_clubangle = -90;       % Club angle range (degrees)
cfg.parametric.max_clubangle = -80;
```

### Initial Conditions

```matlab
cfg.initial.alpha0 = deg2rad(-124);       % Initial wrist cock angle
cfg.initial.theta0 = deg2rad(-166);       % Initial arm angle (backswing)
```

## Output

### Generated Plots

| Filename | Description |
|----------|-------------|
| `GolfSwingPlot.png` | 2D swing trajectory with joint markers |
| `TimeDomainPlot.png` | Arm angle, club angle, speed vs time |
| `ImpactPlot.png` | Club angle and speed vs distance to ball |
| `TorquePlot.png` | Shoulder and wrist torque profiles |
| `ParametricAnalysisPlot.png` | Impact metrics vs club mass |
| `ParametricSwingsPlot.png` | Tiled swing trajectories |
| `ComparativeSwingsPlot.png` | Heavy vs light club overlay |

### Console Output

```
=== Golf Swing MBD Simulation ===

Physical Parameters:
  Arm length:    0.615 m
  Club length:   1.105 m
  Initial mass:  0.394 kg
  Shoulder torque: 250.0 N*m

Running parametric study...
  Shot 1: mc = 0.394 kg, v = 42.3 m/s, angle = -85.2 deg
  Shot 2: mc = 0.389 kg, v = 43.1 m/s, angle = -84.8 deg
  ...
  Stopping: swing with mc = 0.334 kg is unrealistic

Completed: 12 realistic swings found

=== Results Summary ===
Mass range: 0.394 to 0.339 kg
Speed range: 42.3 to 48.7 m/s
Time range: 178.2 to 165.4 ms
```

## Theory

### Why Does Club Mass Matter?

The simulation reveals key biomechanical insights:

1. **Lighter clubs = Higher head speed**: Reduced mass allows faster angular acceleration with the same applied torque

2. **Timing changes**: The optimal wrist release timing shifts with club mass due to changes in system inertia

3. **Trade-offs exist**: While lighter clubs achieve higher speeds, very light clubs can produce unrealistic swing patterns

### The Delayed Release

Professional golfers maintain "wrist cock" during the early downswing, releasing it near impact. This is modeled by the wrist torque switching from resistive to free at time t_release.

The optimal release time depends on:
- System inertia (k)
- Difference between arm and club inertias (k₀)
- Angular momentum coupling (k₁)

### Impact Detection

The simulation detects "impact" when the club head x-coordinate crosses the ball position. At this instant, it records:
- Club head speed (critical for ball flight distance)
- Club angle (affects launch angle and spin)
- Downswing duration

## References

1. Cochran, A. & Stobbs, J. (1968). *The Search for the Perfect Swing*. Lippincott.

2. Jorgensen, T.P. (1994). *The Physics of Golf*. Springer-Verlag.

3. Penner, A.R. (2003). "The physics of golf". *Reports on Progress in Physics*, 66, 131-171.

4. Nesbit, S.M. (2005). "A three dimensional kinematic and kinetic study of the golf swing". *Journal of Sports Science and Medicine*, 4, 499-519.

---

*This project was developed as a course project for Multibody Dynamics.*
