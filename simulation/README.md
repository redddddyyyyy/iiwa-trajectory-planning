# Gazebo MVP Demo (48-Hour Interview Build)

This folder gives you a fast, interview-ready simulation path for replaying your generated iiwa trajectory in ROS2/Gazebo.

## What You Get

- ROS2 package: `iiwa_trajectory_demo`
- Replay node that publishes your Nx7 MATLAB trajectory as `trajectory_msgs/JointTrajectory`
- Launch file for one-command trajectory replay to your controller topic

## Assumptions

- You already have a working iiwa simulation in Gazebo (or RViz) with a running joint trajectory controller.
- Controller command topic accepts `trajectory_msgs/JointTrajectory`.
- Joint order in controller matches trajectory columns.

Default joint order used by replay node:
- `A1 A2 A3 A4 A5 A6 A7`

## Build

```bash
cd /path/to/iiwa-trajectory-planning/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run (after your Gazebo/ros2_control stack is already running)

```bash
ros2 launch iiwa_trajectory_demo iiwa_replay.launch.py \
  trajectory_file:=/absolute/path/to/iiwa-trajectory-planning/data/trajectory_smooth_4001x7.txt \
  controller_topic:=/joint_trajectory_controller/joint_trajectory \
  time_scale:=1.0
```

## Quick Verification

1. Confirm controller topic exists:
```bash
ros2 topic list | grep joint_trajectory_controller
```

2. Confirm message type:
```bash
ros2 topic type /joint_trajectory_controller/joint_trajectory
```
Expected: `trajectory_msgs/msg/JointTrajectory`

3. Verify command publication:
```bash
ros2 topic echo /joint_trajectory_controller/joint_trajectory --once
```

## Interview-Day Demo Flow (recommended)

1. Start Gazebo world + iiwa controller stack.
2. Run smooth trajectory replay (`trajectory_smooth_4001x7.txt`).
3. Replay raw trajectory (`trajectory_raw_4001x7.txt`) at lower speed, e.g. `time_scale:=0.5`.
4. Show your before/after plots from `assets/`.
5. Explain: sparse-waypoint IK + cubic time-scaling reduced extreme spikes.

## Fast Recording Command (optional)

Use your platform recorder while replaying. Keep final clip 20-30 seconds with:
- initial robot pose
- trajectory execution
- one plot screenshot (raw vs smooth)

## Notes

- Replay node publishes the full trajectory once after a short startup delay.
- If the robot does not move, check topic name and joint ordering first.
