# CLAUDE.md — iiwa Trajectory Planning Project

Full context for Claude Code sessions on this project.

---

## Project Overview

**Goal:** Generate smooth, physically realistic joint-space trajectories for the KUKA LBR iiwa 7 R800 manipulator, simulate them in ROS2 Gazebo, and demonstrate a full pick-and-place task using MoveIt2.

**Core problem solved:** Dense Cartesian IK sampling causes the solver to jump between solution branches, producing violent velocity spikes (up to 348 rad/s). The fix: solve IK only at 9 sparse waypoints, then apply cubic polynomial interpolation.

**Results:**
- Peak joint velocity: 348.56 → 22.90 rad/s (**93.4% reduction**)
- Peak joint acceleration: 34,877 → 2,249 rad/s² (**93.6% reduction**)

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
├── src/                                 ← MATLAB scripts (reference only)
│   ├── main_smooth_joint_traj.m
│   └── main_raw_cartesian_ik.m
├── assets/                              ← plots and result images
├── docs/                                ← technical report PDFs
├── CLAUDE.md                            ← this file
└── memory.md                            ← session notes

~/ros2_iiwa_ws/                          ← iiwa sim stack (controllers, MoveIt)
└── src/
    ├── iiwa_ros2/                       ← robot URDF, controllers, MoveIt config
    └── ros-controls/
```

---

## Critical Configuration Facts

- **Robot base frame:** `iiwa_base` is at **world x=1.0** (confirmed via `tf2_echo world tool0`)
  - All Gazebo spawns, MoveIt collision objects, and motion goals must use `x=1.0` as the robot's x position
  - Objects spawned at `x=0.0` are **1 meter behind** the robot — unreachable
- **Planning group name:** `iiwa_arm`
- **End-effector link:** `tool0`
- **Joint names:** `joint_a1` through `joint_a7`
- **Controller topic:** `/iiwa_arm_controller/joint_trajectory`
- **Planning pipelines available:** `ompl` (default: RRTConnect) and `pilz`
- **MoveIt action server:** `/move_action`
- **Apply planning scene service:** `/apply_planning_scene`

---

## Gazebo Object Positions (world frame, x=1.0 offset applied)

| Object | x | y | z (centre) | Size |
|--------|---|---|-------------|------|
| demo_table | 1.0 | -0.6 | 0.375 | 1.0 × 0.8 × 0.75 m |
| target_block | 1.0 | -0.3 | 0.78 | 0.06 × 0.06 × 0.06 m |

Table top surface: **z = 0.75 m**
Block top surface: **z = 0.81 m**

---

## SDF Files (already at /tmp from previous sessions)

These persist across reboots only until `/tmp` is cleared. If missing, re-create them:

```bash
# Check if SDF files exist
ls /tmp/demo_table.sdf /tmp/target_block.sdf
```

If missing, the SDF content is embedded in `pick_and_place.py` (`TARGET_BLOCK_SDF` constant). Table SDF must be re-created manually (see Known Working Commands below).

---

## How to Run — Full Session Startup

### Terminal 1 — Launch Planning Stack (keep open always)
```bash
cd ~/ros2_iiwa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.sh
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true start_rviz:=true use_planning:=true
```
Wait until you see:
- Gazebo opens with the robot arm visible
- RViz opens
- Log shows: `iiwa_arm_controller` configured and activated
- Log shows: `You can start planning now!`

---

### Terminal 2 — Spawn Table and Block in Gazebo
Run **only after robot is visible in Gazebo**:
```bash
source /opt/ros/humble/setup.bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: demo_table}"
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: target_block}"
ros2 run gazebo_ros spawn_entity.py -entity demo_table -file /tmp/demo_table.sdf -x 1.0 -y -0.6 -z 0.375
ros2 run gazebo_ros spawn_entity.py -entity target_block -file /tmp/target_block.sdf -x 1.0 -y -0.3 -z 0.78
```

---

### Terminal 3 — Add Collision Objects to MoveIt
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo scene_setup
```
Expected output: `SUCCESS — demo_table and target_block added to MoveIt planning scene.`
Objects appear as green boxes in RViz (requires MotionPlanning display — see RViz Setup below).

---

### Terminal 4 — Pick and Place Demo
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo pick_and_place
```
The arm executes this 7-step sequence automatically:
1. Move to APPROACH (above block, z=0.92)
2. Move to GRASP (at block, z=0.85)
3. GRIP — block attaches to tool0 in RViz
4. LIFT — arm rises carrying the block
5. Move to PLACE (y=-0.5, different table spot)
6. RELEASE — block detaches + re-spawns in Gazebo at new position
7. RETREAT — arm moves clear

---

### Alternative: Just move end-effector to target (no pick-and-place)
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo move_to_target
```

---

### Alternative: Replay raw trajectory
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo trajectory_replay --ros-args \
  -p use_sim_time:=true \
  -p trajectory_file:=/home/reddy/iiwa/data/trajectory_raw_4001x7.txt \
  -p controller_topic:=/iiwa_arm_controller/joint_trajectory \
  -p joint_names:="[joint_a1,joint_a2,joint_a3,joint_a4,joint_a5,joint_a6,joint_a7]" \
  -p start_delay_sec:=3.0 \
  -p time_scale:=2.0
```

---

## RViz Setup

After RViz opens, to see collision objects:
1. Click **Add** (bottom-left Displays panel)
2. Choose **MotionPlanning** → OK
3. Expand **MotionPlanning → Planning Scene** → tick **Show World Geometry**

To see the robot model only: **RobotModel** display should already be present.

---

## Building the ROS2 Package

After any code change to `simulation/ros2_ws/`:
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select iiwa_trajectory_demo
```

To rebuild the full iiwa sim stack (only needed if stack source changes):
```bash
cd ~/ros2_iiwa_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

---

## What Was Completed (Session History)

### Session 1 (previous)
- Built both ROS2 workspaces successfully
- Fixed non-motion issue: missing smooth file + topic/joint name mismatches
- Confirmed `iiwa_arm_controller` active
- Trajectory replay node working with raw file
- Spawned table + block in Gazebo

### Session 2 (current)
- Identified correct launch file: `iiwa.launch.py` with `use_planning:=true`
  - Previous sessions used `iiwa_planning.launch.py` alone (no Gazebo, no robot spawn)
- Discovered robot base is at **world x=1.0** — all objects needed x offset
- Fixed collision objects and motion goals to use correct positions
- Wrote and tested `scene_setup.py` — table and block now visible in RViz
- Wrote and tested `move_to_target.py` — end-effector successfully reaches above block
- Wrote `pick_and_place.py` — full pick → carry → place sequence

---

## Known Issues / Debugging

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot not visible in RViz/Gazebo | Wrong launch file | Use `iiwa.launch.py` not `iiwa_planning.launch.py` |
| "No ContextLoader for planner_id" | pipeline_id/planner_id not set | Set `pipeline_id='ompl'`, `planner_id='RRTConnect'` |
| "Unable to sample valid states for goal tree" | Goal out of reach or orientation constraint too tight | Remove orientation constraint; check x=1.0 offset |
| Collision objects not in RViz | ApplyPlanningScene response not awaited | Use `rclpy.spin()` + `raise SystemExit` in callback |
| Planning fails (error code 99999) | Goal position unreachable | Verify x=1.0 offset; lower z height |
| scene_setup exits with no SUCCESS | `spin_once` exits before async response | Use `rclpy.spin()` with `raise SystemExit` in callback |

---

## Next Steps

1. **Test pick_and_place.py** — verify full sequence works end-to-end
2. **Generate smooth trajectory** — requires MATLAB R2023a+ with Robotics System Toolbox
   - Run `src/main_smooth_joint_traj.m` to produce `data/trajectory_smooth_4001x7.txt`
   - Then replay with `trajectory_replay` node
3. **Add visual gripper** — install `gazebo_ros_link_attacher` for physics-based gripping
   ```bash
   sudo apt install ros-humble-gazebo-ros-link-attacher
   ```
   Then update `pick_and_place.py` to use `/link_attacher_node/attach` service instead of MoveIt-only attach
4. **Compare raw vs smooth** in Gazebo — run both trajectories and visually compare motion quality
5. **Tune place position** — if `pick_and_place` fails at PLACE step, adjust `PLACE` waypoint in `pick_and_place.py` (currently `(1.0, -0.5, 0.92)`)

---

## Package Entry Points

All scripts are in `simulation/ros2_ws/src/iiwa_trajectory_demo/iiwa_trajectory_demo/`:

| Command | Script | Purpose |
|---------|--------|---------|
| `ros2 run iiwa_trajectory_demo trajectory_replay` | `trajectory_replay.py` | Replay Nx7 file to controller |
| `ros2 run iiwa_trajectory_demo scene_setup` | `scene_setup.py` | Add table+block to MoveIt |
| `ros2 run iiwa_trajectory_demo move_to_target` | `move_to_target.py` | Move end-effector to target |
| `ros2 run iiwa_trajectory_demo pick_and_place` | `pick_and_place.py` | Full pick-and-place demo |
