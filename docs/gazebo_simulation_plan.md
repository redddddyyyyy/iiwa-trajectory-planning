# Gazebo Simulation Plan

## Objective

Add a simulation showcase that demonstrates the generated trajectories on a virtual KUKA iiwa arm with reproducible setup steps and visual outputs suitable for portfolio use.

## Scope

- Simulate the iiwa model in Gazebo.
- Replay `data/trajectory_smooth_4001x7.txt` and `data/trajectory_raw_4001x7.txt`.
- Record side-by-side videos for profile presentation.
- Add reproducible launch and playback instructions.

## Milestones

1. Robot and world setup
- Add Gazebo-compatible iiwa model and a minimal workcell world.
- Keep coordinate frame definitions aligned with MATLAB scripts.

2. Trajectory replay node
- Build a ROS/Gazebo bridge node to stream joint targets from Nx7 trajectory files.
- Support fixed-rate playback (200 Hz nominal) and variable speed factor for demos.

3. Validation tooling
- Log joint states from simulation.
- Compare commanded vs simulated trajectories and compute tracking errors.

4. Demo deliverables
- Export one concise portfolio video of raw vs smooth playback.
- Add GIF snippet and full video link in README.

## Suggested Technical Stack

- ROS 2 + Gazebo Harmonic (or Ignition equivalent in your environment)
- `ros2_control` for joint command interfaces
- Python replay script for trajectory streaming and logging

## Definition of Done

- One command to launch simulation.
- One command to replay a selected trajectory.
- README section with setup and demo commands.
- Portfolio-ready visual artifacts committed under `assets/`.
