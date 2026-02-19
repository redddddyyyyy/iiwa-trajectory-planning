# KUKA iiwa 7-DOF Trajectory Planning

[![CI](https://github.com/redddddyyyyy/iiwa-trajectory-planning/actions/workflows/ci.yml/badge.svg)](https://github.com/redddddyyyyy/iiwa-trajectory-planning/actions/workflows/ci.yml)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange?style=flat&logo=mathworks)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.11%2B-blue?style=flat&logo=python)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Smooth joint-space trajectory generation for a **KUKA LBR iiwa 7 R800** manipulator, eliminating IK branch-jump discontinuities that create unrealistic velocity spikes.

## Demo Preview

A short simulation video/GIF is planned as part of the Gazebo integration. Current raw vs smoothed behavior is shown below.

![Demo Preview](assets/demo_preview_raw_vs_smooth.png)

## Key Results

| Metric | Before | After | Improvement |
|--------|-------:|------:|:-----------:|
| Peak Joint Velocity (rad/s) | 348.56 | 22.90 | **93.4%** |
| Peak Joint Acceleration (rad/s²) | 34,877.90 | 2,249.08 | **93.6%** |

## Why This Matters

Dense Cartesian interpolation with pointwise IK can switch IK branches between adjacent samples. That creates physically impossible joint velocities and unstable command streams.

This project solves IK only at sparse waypoints, then time-scales a joint-space trajectory for smooth and controller-ready motion.

## Quick Start

### Requirements

- MATLAB R2023a+ with Robotics System Toolbox
- Python 3.11+ for analysis utilities

Notes:
- Scripts first try `loadrobot("kukaIiwa7")`.
- If your MATLAB install does not include that model, add `iiwa7.urdf` at repo root and scripts will fall back to `importrobot("iiwa7.urdf")`.

### Run MATLAB

```matlab
% Smooth trajectory (recommended)
run('src/main_smooth_joint_traj.m')

% Raw baseline (for comparison)
run('src/main_raw_cartesian_ik.m')
```

### Analyze with Python

```bash
python -m pip install -r requirements.txt
python scripts/analyze_trajectory.py data/trajectory_raw_4001x7.txt --dt 0.005 --outdir assets
```

### Outputs

- `data/trajectory_smooth_4001x7.txt`: smoothed controller-ready joint trajectory (200 Hz)
- `data/trajectory_raw_4001x7.txt`: raw baseline trajectory
- `assets/*.png`, `assets/*.json`: plots and summary statistics

## Method Overview

1. Define placement task over a 4-target marker grid with fixed end-effector orientation.
2. Compute calibration transforms (Base, TCP, Camera, Marker).
3. Solve IK only at 9 sparse waypoints to reduce branch-jumps.
4. Generate a smooth 200 Hz joint trajectory using cubic polynomial time-scaling.
5. Validate feasibility via joint angle, velocity, and acceleration plots.

## Repository Structure

```text
├── src/                    MATLAB trajectory generation scripts
├── data/                   Exported trajectories (Nx7)
├── assets/                 Plots and metrics used in README
├── docs/                   Report PDFs and roadmap notes
├── simulation/             ROS2/Gazebo replay MVP (interview-ready)
├── scripts/                Python analysis utilities
├── .github/workflows/      CI checks
└── requirements.txt        Python dependencies for analysis
```

## Next Step: Gazebo Simulation

MVP replay tooling is now added under `simulation/` so trajectories can be replayed to a ROS2 joint trajectory controller in Gazebo.

### Run This on Your ROS2 Machine (Next)

```bash
cd /path/to/iiwa-trajectory-planning/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch iiwa_trajectory_demo iiwa_replay.launch.py \
  trajectory_file:=/path/to/iiwa-trajectory-planning/data/trajectory_smooth_4001x7.txt \
  controller_topic:=/joint_trajectory_controller/joint_trajectory \
  time_scale:=1.0
```

If your controller topic differs, replace `controller_topic` with your active joint trajectory topic.

Runbook: [`simulation/README.md`](simulation/README.md)

Roadmap: [`docs/gazebo_simulation_plan.md`](docs/gazebo_simulation_plan.md)

## Documentation

- [`docs/technical_report_original.pdf`](docs/technical_report_original.pdf): full technical report
- [`docs/results_original.pdf`](docs/results_original.pdf): detailed results analysis

## License

MIT
