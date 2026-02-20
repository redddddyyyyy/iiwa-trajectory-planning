# CLAUDE.md — iiwa Trajectory Planning Project

Full context for Claude Code sessions on this project.

---

## Project Overview

**Goal:** Generate smooth, physically realistic joint-space trajectories for the KUKA LBR iiwa 7 R800 manipulator, simulate them in ROS2 Gazebo, and demonstrate a full pick-and-place task using MoveIt2.

**Core problem solved:** Dense Cartesian IK sampling (original `main.m`) causes the solver to jump between solution branches, producing violent velocity spikes (up to 348 rad/s). The fix: solve IK only at 9 sparse waypoints, then apply cubic polynomial interpolation.

**Performance gains:**
- Peak joint velocity: 348.56 → 22.90 rad/s (**93.4% reduction**)
- Peak joint acceleration: 34,877 → 2,249 rad/s² (**93.6% reduction**)

---

## Original MATLAB Code (from final project zip)

The original `main.m` (submitted for course) does the following:
- Loads `iiwa7.urdf` via `importrobot()`
- Defines calibration transforms: TCP, camera-end-effector (`Tec`), camera-marker (`Tca`)
- Photo pose `qc_photo` in joint space (used to recover base→marker transform)
- 4 marker targets in a grid: `markerOffsets` with `dx=70mm`, `dy=73mm`
- 9 waypoints: 1 photo pose + 2 per target (safe-Z approach at `tableZ+0.12m` + contact)
- **Dense Cartesian interpolation**: interpolates all 9 waypoints over 20 seconds at 5ms steps (4001 samples), solves IK at every sample — this is the problem
- Saves raw trajectory to `singareddy rajeev.txt` (7 columns, same format as `trajectory_raw_4001x7.txt`)
- This raw trajectory file IS the baseline `data/trajectory_raw_4001x7.txt` in the repo

The improved script `src/main_smooth_joint_traj.m` solves IK only at the 9 sparse waypoints, then applies cubic polynomial interpolation between joint configs (not Cartesian), eliminating branch-jump discontinuities.

---

## Environment

| Item | Value |
|------|-------|
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| Simulator | Gazebo Classic |
| Robot | KUKA LBR iiwa 7 R800 |
| MATLAB | Not available on this machine |
| Python | 3.11+ |

---

## Repository Structure

```
/home/reddy/iiwa/                        ← main project repo
├── data/
│   ├── trajectory_raw_4001x7.txt        ← baseline raw trajectory (works)
│   └── trajectory_smooth_4001x7.txt     ← MISSING (needs MATLAB to generate)
├── scripts/
│   └── analyze_trajectory.py            ← velocity/accel analysis + plots
├── simulation/
│   └── ros2_ws/                         ← ROS2 package workspace
│       └── src/iiwa_trajectory_demo/
│           └── iiwa_trajectory_demo/
│               ├── trajectory_replay.py ← replay Nx7 file to controller
│               ├── scene_setup.py       ← add table+block to MoveIt scene
│               ├── move_to_target.py    ← move end-effector above block
│               └── pick_and_place.py    ← full pick → carry → place sequence
├── src/
│   ├── main_smooth_joint_traj.m         ← sparse-waypoint IK + cubic interp
│   └── main_raw_cartesian_ik.m          ← dense IK baseline (shows problem)
├── assets/                              ← plots and result images
├── docs/                                ← technical report PDFs
├── CLAUDE.md                            ← this file
└── memory.md                            ← session notes (not committed)

~/ros2_iiwa_ws/                          ← iiwa sim stack (controllers, MoveIt)
└── src/
    ├── iiwa_ros2/
    └── ros-controls/
```

---

## Critical Configuration Facts

- **Robot base frame:** `iiwa_base` is at **world x=1.0** (confirmed via `tf2_echo world tool0`)
  - All Gazebo spawns, MoveIt collision objects, and motion goals must use `x=1.0`
  - Objects spawned at `x=0.0` are 1 meter behind the robot — unreachable
- **tool0 current pose at home:** Translation `[0.221, 0.000, 0.428]`, Rotation RPY `[0, -90°, 0]`
  - tool0 z-axis points in world **-X direction** (not downward) — don't use tool0-frame offsets
- **Planning group:** `iiwa_arm`
- **End-effector link:** `tool0`
- **Joint names:** `joint_a1` through `joint_a7`
- **Controller topic:** `/iiwa_arm_controller/joint_trajectory`
- **Planning pipelines:** `ompl` (RRTConnect) and `pilz` (PTP/LIN)
- **MoveIt action server:** `/move_action`
- **Apply planning scene service:** `/apply_planning_scene`

---

## Gazebo Object Positions (world frame, x=1.0 offset applied)

| Object | x | y | z (centre) | Size |
|--------|---|---|------------|------|
| demo_table | 1.0 | -0.6 | 0.375 | 1.0 × 0.8 × 0.75 m |
| target_block | 1.0 | -0.3 | 0.78 | 0.06 × 0.06 × 0.06 m |

Table top surface: **z = 0.75 m** | Block top surface: **z = 0.81 m**

---

## Pick-and-Place Waypoints

| Step | Waypoint | World (x, y, z) | Notes |
|------|----------|-----------------|-------|
| 1 | APPROACH | (1.0, -0.3, 0.92) | above block |
| 2 | GRASP | (1.0, -0.3, 0.85) | just above block top |
| 3 | GRIP | — | attaches block to tool0 in MoveIt |
| 4 | LIFT | (1.0, -0.3, 0.95) | clears table edge |
| 5 | PLACE | (1.0, -0.5, 0.90) | different table spot |
| 6 | RELEASE | — | detach + Gazebo re-spawn at (1.0, -0.5, 0.78) |
| 7 | RETREAT | HOME joints | OMPL to SRDF ready state |

**Path constraint (all moves):** `tool0` must stay above z=0.80 throughout to prevent elbow-down configurations that sweep through the table.

---

## SDF Files (in /tmp, re-create if missing after reboot)

```bash
cat > /tmp/demo_table.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="demo_table"><static>true</static>
    <link name="link">
      <collision name="collision"><geometry><box><size>1.0 0.8 0.75</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>1.0 0.8 0.75</size></box></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient></material></visual>
    </link>
  </model>
</sdf>
EOF

cat > /tmp/target_block.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="target_block"><static>true</static>
    <link name="link">
      <collision name="collision"><geometry><box><size>0.06 0.06 0.06</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.06 0.06 0.06</size></box></geometry>
        <material><ambient>0.9 0.2 0.2 1</ambient></material></visual>
    </link>
  </model>
</sdf>
EOF
```

---

## How to Run — Full Session Startup

### Terminal 1 — Launch Planning Stack (keep open always)
```bash
cd ~/ros2_iiwa_ws
source /opt/ros/humble/setup.bash && source install/setup.bash && source /usr/share/gazebo/setup.sh
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true start_rviz:=true use_planning:=true
```
Wait for: robot visible in Gazebo + RViz + `You can start planning now!` in logs.

### Terminal 2 — Spawn Table and Block in Gazebo
```bash
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity demo_table  -file /tmp/demo_table.sdf  -x 1.0 -y -0.6 -z 0.375
ros2 run gazebo_ros spawn_entity.py -entity target_block -file /tmp/target_block.sdf -x 1.0 -y -0.3 -z 0.78
```

### Terminal 3 — Add Collision Objects to MoveIt
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo scene_setup
```
Expected: `SUCCESS — demo_table and target_block added to MoveIt planning scene.`

### Terminal 4 — Pick and Place
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo pick_and_place
```

### Alternative: Move end-effector to target only
```bash
ros2 run iiwa_trajectory_demo move_to_target
```

### Alternative: Replay raw trajectory
```bash
ros2 run iiwa_trajectory_demo trajectory_replay --ros-args \
  -p use_sim_time:=true \
  -p trajectory_file:=/home/reddy/iiwa/data/trajectory_raw_4001x7.txt \
  -p controller_topic:=/iiwa_arm_controller/joint_trajectory \
  -p joint_names:="[joint_a1,joint_a2,joint_a3,joint_a4,joint_a5,joint_a6,joint_a7]" \
  -p start_delay_sec:=3.0 \
  -p time_scale:=2.0
```

---

## Building the ROS2 Package

```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select iiwa_trajectory_demo
```

Full iiwa stack rebuild (only if stack source changes):
```bash
cd ~/ros2_iiwa_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

---

## RViz Setup

1. Click **Add** (bottom-left) → **MotionPlanning** → OK
2. Expand **MotionPlanning → Planning Scene** → tick **Show World Geometry**

---

## Session History

### Session 1
- Built both workspaces, fixed non-motion issue (wrong topic/joint names)
- Confirmed `iiwa_arm_controller` active, trajectory replay working

### Session 2
- Fixed launch file: `iiwa.launch.py use_planning:=true` (not `iiwa_planning.launch.py` alone)
- Discovered robot base at **world x=1.0** — critical offset for all goals/objects
- Built and tested: `scene_setup`, `move_to_target`, `pick_and_place`
- Fixed ghost timer bug (`.cancel` missing `()`) causing infinite step 1 loop
- Fixed GRIP failure: use world frame for attached object (tool0 rotated -90° around Y)
- Fixed RETREAT: Pilz PTP fails post-PLACE; switched to OMPL joint goal
- Added path constraint (z > 0.80) to prevent elbow-down sweeping through table
- Read original `main.m` from final project zip — confirms raw trajectory method

---

## Known Issues / Debugging

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot not visible | Wrong launch file | Use `iiwa.launch.py` not `iiwa_planning.launch.py` |
| `No ContextLoader for planner_id` | pipeline_id/planner_id not set | Set `pipeline_id='ompl'`, `planner_id='RRTConnect'` |
| `Unable to sample valid states` | Goal out of reach or orientation too tight | Remove orientation constraint; verify x=1.0 offset |
| scene_setup exits with no SUCCESS | spin_once exits before async response | Use `rclpy.spin()` + `raise SystemExit` in callback |
| Planning fails error 99999 | Goal unreachable or start state in collision | Restart everything; verify x=1.0 offset |
| GRIP fails (ApplyPlanningScene=False) | Attached object in tool0 frame — wrong direction | Use world frame for attached object position |
| RETREAT fails error -16 | Pilz PTP can't reach home from post-PLACE config | Use OMPL with joint goal for retreat |
| Arm sweeps through/under table | OMPL picks elbow-down path | Path constraint: tool0 z > 0.80 at all times |
| Step 1 spams infinitely | Ghost timer `.cancel` (missing `()`) | Fixed — `_busy` flag + proper cancel |

---

## Next Steps

1. **Test path constraint** — verify arm stays above table during all transit moves
2. **Generate smooth trajectory** — requires MATLAB R2023a+ with Robotics System Toolbox
   - Run `src/main_smooth_joint_traj.m` → produces `data/trajectory_smooth_4001x7.txt`
   - Replay with `trajectory_replay` node
3. **Visual gripper** — install `gazebo_ros_link_attacher` for physics-based gripping:
   ```bash
   sudo apt install ros-humble-gazebo-ros-link-attacher
   ```
4. **Compare raw vs smooth** trajectories in Gazebo simulation

---

## Package Entry Points

| Command | Script | Purpose |
|---------|--------|---------|
| `ros2 run iiwa_trajectory_demo trajectory_replay` | `trajectory_replay.py` | Replay Nx7 file to controller |
| `ros2 run iiwa_trajectory_demo scene_setup` | `scene_setup.py` | Add table+block to MoveIt |
| `ros2 run iiwa_trajectory_demo move_to_target` | `move_to_target.py` | Move end-effector to target |
| `ros2 run iiwa_trajectory_demo pick_and_place` | `pick_and_place.py` | Full pick-and-place demo |
