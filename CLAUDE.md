# CLAUDE.md — iiwa Trajectory Planning Project

Full context for Claude Code sessions on this project.

---

## Project Overview

**Goal:** Generate smooth, physically realistic joint-space trajectories for the KUKA LBR iiwa 7 R800 manipulator, simulate them in ROS2 Gazebo, and demonstrate a full pick-and-place task using MoveIt2.

**Core problem solved:** Dense Cartesian IK sampling (original `main.m`) causes the solver to jump between solution branches, producing violent velocity spikes. The fix: extract 9 sparse boundary waypoints from the raw trajectory, apply cubic polynomial interpolation in joint space.

**Performance gains (confirmed on this machine):**
- Peak joint velocity: 696.66 → 1.71 rad/s (**99.8% reduction**)
- Peak joint acceleration: 139,377 → 2.3 rad/s² (**~100% reduction**)

---

## Original MATLAB Code (from final project zip)

The original `main.m` (submitted for course, output confirmed byte-identical to `data/trajectory_raw_4001x7.txt`):
- Loads `iiwa7.urdf` via `importrobot()`
- Defines calibration transforms: TCP, camera-end-effector (`Tec`), camera-marker (`Tca`)
- Photo pose `qc_photo` in joint space (used to recover base→marker transform)
- 4 marker targets in a grid: `markerOffsets` with `dx=70mm`, `dy=73mm`
- 9 waypoints: 1 photo pose + 2 per target (safe-Z approach at `tableZ+0.12m` + contact)
- **Dense Cartesian interpolation**: 9 waypoints over 20 s at 5 ms steps (4001 samples), IK at every sample — this causes branch-jump velocity spikes
- Saves raw trajectory to `singareddy rajeev.txt` — **confirmed byte-identical** to `data/trajectory_raw_4001x7.txt`

**Python equivalent of smooth script:** `scripts/generate_smooth_trajectory.py`
- Extracts same 9 boundary waypoints (indices 0,500,…,4000) from raw trajectory
- Applies `scipy.CubicSpline` with `bc_type='clamped'` (zero velocity at endpoints)
- Matches MATLAB `cubicpolytraj` with `VelocityBoundaryCondition=zeros(7,2)`
- Output already generated at `data/trajectory_smooth_4001x7.txt`

---

## Environment

| Item | Value |
|------|-------|
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| Simulator | Gazebo Classic |
| Robot | KUKA LBR iiwa 7 R800 |
| MATLAB | Not available on this machine |
| Python | 3.10 (system) |
| scipy | 1.15.3 via `pip3 install --user --upgrade scipy` (system 1.8.0 is binary-incompatible with numpy 2.x) |

---

## Repository Structure

```
/home/reddy/iiwa/
├── data/
│   ├── trajectory_raw_4001x7.txt        ← MATLAB output baseline (696 rad/s peak)
│   └── trajectory_smooth_4001x7.txt     ← Python cubic spline output (1.71 rad/s peak)
├── scripts/
│   ├── analyze_trajectory.py            ← velocity/accel analysis + plots
│   └── generate_smooth_trajectory.py    ← generates trajectory_smooth_4001x7.txt
├── simulation/
│   └── ros2_ws/
│       └── src/iiwa_trajectory_demo/
│           └── iiwa_trajectory_demo/
│               ├── trajectory_replay.py ← replay Nx7 file to controller
│               ├── scene_setup.py       ← add table+block to MoveIt scene
│               ├── move_to_target.py    ← move end-effector above block
│               └── pick_and_place.py    ← full 7-step pick → carry → place
├── src/
│   ├── main_smooth_joint_traj.m         ← MATLAB sparse-waypoint IK + cubic interp
│   └── main_raw_cartesian_ik.m          ← MATLAB dense IK baseline (shows problem)
├── assets/                              ← plots and result images
├── docs/                                ← technical report PDFs
└── CLAUDE.md                            ← this file

~/ros2_iiwa_ws/                          ← iiwa sim stack (controllers, MoveIt)
└── src/
    ├── iiwa_ros2/
    └── ros-controls/
```

---

## Critical Configuration Facts

- **Robot base frame:** `iiwa_base` is at **world x=1.0** (confirmed via `tf2_echo world tool0`)
  - All Gazebo spawns, MoveIt goals, and collision objects must use `x=1.0`
  - Objects at `x=0.0` are 1 m behind the robot — unreachable
- **tool0 at home:** Translation `[0.221, 0.000, 0.428]`, RPY `[0, -90°, 0]`
  - tool0 z-axis points in world **-X direction** (not downward) — never use tool0-frame offsets
- **Planning group:** `iiwa_arm`
- **End-effector link:** `tool0`
- **Joint names:** `joint_a1` through `joint_a7`
- **Controller topic:** `/iiwa_arm_controller/joint_trajectory`
- **Planning pipelines:** `ompl` (RRTConnect), `pilz` (PTP/LIN)
- **MoveIt action server:** `/move_action`
- **Cartesian path service:** `/compute_cartesian_path` (`moveit_msgs/srv/GetCartesianPath`)
- **Execute trajectory action:** `/execute_trajectory` (`moveit_msgs/action/ExecuteTrajectory`)
- **Apply planning scene service:** `/apply_planning_scene`

---

## Gazebo Object Positions (world frame)

| Object | x | y | z (centre) | Size |
|--------|---|---|------------|------|
| demo_table | 1.0 | -0.6 | 0.375 | 1.0 × 0.8 × 0.75 m |
| target_block | 1.0 | -0.3 | 0.78 | 0.06 × 0.06 × 0.06 m |

Table top: **z = 0.75 m** | Block top: **z = 0.81 m** | Block centre at pick: **z = 0.78 m**

---

## Pick-and-Place — Full Working 7-Step Sequence

**Status: FULLY WORKING** (confirmed Session 6)

| Step | Name | Target (world) | Planner | Notes |
|------|------|----------------|---------|-------|
| 1 | APPROACH | (1.0, -0.3, 0.92) | OMPL RRTConnect | Free motion from home to above block |
| 2 | GRASP | (1.0, -0.3, 0.85) | **Cartesian** | Straight-line descent; orientation locked from TF after step 1 |
| 3 | GRIP | — | ApplyPlanningScene | Two-stage: (A) REMOVE from world, (B) ACO attach to tool0 |
| 4 | CARRY | (1.0, -0.5, 0.90) | OMPL RRTConnect | Single goal; clean short 0.2 m move, no detours |
| 5 | RELEASE | — | ApplyPlanningScene | Atomic: ACO detach + world ADD at place + Gazebo delete+respawn |
| 6 | ASCENT | current x,y → z=0.95 | **Cartesian** | Straight up to safe height before OMPL retreat |
| 7 | RETREAT | HOME joints | OMPL RRTConnect | Joint goal to SRDF ready state |

**Design decisions (hard-won):**
- Cartesian ONLY for short vertical segments (steps 2, 6) — orientation lock is reliable for vertical
- OMPL for all horizontal/free-space moves (steps 1, 4, 7)
- Tried Cartesian carry (step 4): failed at 55–63% because OMPL-chosen orientation at GRASP can't be maintained over 0.2 m horizontal span (IK workspace constraint). One OMPL goal is cleaner.
- `jump_threshold=0.0` on all Cartesian paths — `5.0` is a relative threshold that falsely flags singularity-adjacent motion
- `avoid_collisions=True` on ALL Cartesian paths — enabled by removing the block from the world scene BEFORE descent, so the 4 cm gap doesn't trigger false collisions
- ASCENT step (6) required: without it, OMPL at z=0.85 plans paths clipping the table → error_code=-4 CONTROL_FAILED
- Block world-REMOVE moved to `_cartesian_descent()`: removing it before descent lets collision checking stay on; `_grip()` is now a single ACO attach

**Waypoint constants in pick_and_place.py:**
```python
APPROACH     = (1.0, -0.3, 0.92)
GRASP        = (1.0, -0.3, 0.85)
PLACE        = (1.0, -0.5, 0.90)   # OMPL carry target
PLACE_GAZEBO = (1.0, -0.5, 0.78)   # Gazebo block respawn position
ASCENT_Z     = 0.95                 # pre-retreat safe height (in _cartesian_pre_retreat)
```

---

## SDF Files (in /tmp — re-create after reboot only)

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

## HOW TO RUN — Complete Step-by-Step Guide

Open **4 terminals** before starting.

---

### TERMINAL 1 — Launch full simulation stack (keep open always)

```bash
cd ~/ros2_iiwa_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /usr/share/gazebo/setup.sh
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true start_rviz:=true use_planning:=true
```

**Wait for:** `You can start planning now!` — robot visible in both Gazebo and RViz.

---

### TERMINAL 2 — Spawn objects in Gazebo

> SDF files live in `/tmp` — only recreate them after a reboot.

**After reboot only — recreate SDF files:**
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

**Every session — spawn in Gazebo:**
```bash
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity demo_table  -file /tmp/demo_table.sdf  -x 1.0 -y -0.6 -z 0.375
ros2 run gazebo_ros spawn_entity.py -entity target_block -file /tmp/target_block.sdf -x 1.0 -y -0.3 -z 0.78
```

---

### TERMINAL 3 — Add collision objects to MoveIt (every session)

```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run iiwa_trajectory_demo scene_setup
```

**Expected:** `SUCCESS — demo_table and target_block added to MoveIt planning scene.`

---

### TERMINAL 4 — Run a demo

Source once, then run any scenario:
```bash
cd /home/reddy/iiwa/simulation/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
```

#### Scenario A — Full pick-and-place
```bash
ros2 run iiwa_trajectory_demo pick_and_place
```
Expected log (all 7 steps succeed):
```
Step 1/7: Motion succeeded.
Step 2/7: Cartesian descent complete — at GRASP position.
Step 3/7: Block GRIPPED (attached to tool0).
Step 4/7: Motion succeeded.
Step 5/7: Block detached and placed in world scene. Block re-spawned in Gazebo at place position!
Step 6/7: Pre-retreat ascent complete — arm clear of table.
Step 7/7: Motion succeeded.
Pick-and-place sequence COMPLETE!
```

#### Scenario B — Smooth trajectory replay (1.71 rad/s peak)
```bash
ros2 run iiwa_trajectory_demo trajectory_replay --ros-args \
  -p use_sim_time:=true \
  -p controller_topic:=/iiwa_arm_controller/joint_trajectory \
  -p joint_names:="[joint_a1,joint_a2,joint_a3,joint_a4,joint_a5,joint_a6,joint_a7]" \
  -p start_delay_sec:=3.0 \
  -p time_scale:=2.0
```
*(No `trajectory_file` param — defaults to `trajectory_smooth_4001x7.txt`.)*

#### Scenario C — Raw trajectory replay (696 rad/s peak — jerky, for comparison)
```bash
ros2 run iiwa_trajectory_demo trajectory_replay --ros-args \
  -p use_sim_time:=true \
  -p trajectory_file:=/home/reddy/iiwa/data/trajectory_raw_4001x7.txt \
  -p controller_topic:=/iiwa_arm_controller/joint_trajectory \
  -p joint_names:="[joint_a1,joint_a2,joint_a3,joint_a4,joint_a5,joint_a6,joint_a7]" \
  -p start_delay_sec:=3.0 \
  -p time_scale:=2.0
```

#### Scenario D — Quick end-effector test
```bash
ros2 run iiwa_trajectory_demo move_to_target
```

---

## Building the ROS2 Package

Package uses `--symlink-install` so Python edits are picked up immediately.
Rebuild only if colcon metadata is stale:

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

## Regenerating the Smooth Trajectory

```bash
cd /home/reddy/iiwa
python3 scripts/generate_smooth_trajectory.py
```

Prints velocity/acceleration comparison and saves `data/trajectory_smooth_4001x7.txt`.

---

## RViz Setup

1. Click **Add** → **MotionPlanning** → OK
2. Expand **MotionPlanning → Planning Scene** → tick **Show World Geometry**

---

## Session History

### Session 1
- Built both workspaces, fixed wrong topic/joint names, confirmed trajectory replay working.

### Session 2
- Fixed launch file (`iiwa.launch.py use_planning:=true`).
- Discovered robot base at **world x=1.0** — critical for all goals and objects.
- Built and tested `scene_setup`, `move_to_target`, `pick_and_place`.
- Fixed ghost timer bug (`.cancel` missing `()`), GRIP world-frame fix, RETREAT Pilz→OMPL.

### Session 3
- Removed path constraint (z > 0.80) — was blocking APPROACH. All 7 steps complete.
- Remaining bug: block stays on gripper after RELEASE (ApplyPlanningScene not clearing ACO).

### Session 4
- **Fixed RELEASE detach bug**: `_grip()` now two-stage (world REMOVE then ACO attach).
  `_release()` sends one atomic message: ACO REMOVE + world ADD.
- **Generated smooth trajectory** via Python cubic spline (no MATLAB needed).
  `singareddy rajeev.txt` confirmed byte-identical to `trajectory_raw_4001x7.txt`.
  Peak velocity: 696 → 1.71 rad/s (99.8%). scipy 1.15.3 installed via pip.

### Session 5
- **Replaced multi-step OMPL with hybrid Cartesian+OMPL sequence.**
- Added TF listener, GetCartesianPath service, ExecuteTrajectory action.
- Cartesian descent (step 2) works reliably with `jump_threshold=0.0`.
  Setting `jump_threshold=5.0` caused false failures (relative threshold near singularities).
- Cartesian carry (GRASP→LIFT→PLACE) failed at 55–63%: OMPL-chosen orientation can't be
  maintained over 0.2 m horizontal span. Switched step 4 to single OMPL goal.
- RETREAT error_code=-4 (CONTROL_FAILED): arm at z=0.85 too close to table for OMPL.
  Fixed by adding Cartesian pre-retreat ascent (step 6) to z=0.95 before OMPL retreat.
- **Full 7-step sequence working end-to-end.**

### Session 6
- **Fixed OMPL error_code=-4 at step 4 (CARRY):** root cause was MoveIt's cached joint state
  drifting from the controller's actual positions after Cartesian execution.
  Fix: subscribe to `/joint_states` and feed the live positions into `start_state` for every
  OMPL plan request (`_set_start_state()` helper used in `_move_to()` and `_retreat_home()`).
- **Enabled `avoid_collisions=True` for ALL Cartesian paths:**
  Previously descent used `avoid_collisions=False` because the block in the world scene caused
  false collision detections. Fix: move the block world-REMOVE into `_cartesian_descent()` so
  the scene is clear before descending — collision checking now works correctly.
- **Simplified `_grip()`:** block is already removed from world during descent, so GRIP is a
  single ACO attach (no more two-stage world-REMOVE then ACO-attach).
- **Fixed module docstring** to accurately describe the 7-step hybrid sequence.
- **Removed dead `_cartesian_carry()` method** (was never called, referenced undefined constants).
- **Fixed RViz visual bug (block stays on gripper after RELEASE):**
  Set `scene.robot_state.is_diff = True` in `_release()` and split into two sequential
  `ApplyPlanningScene` calls (ACO detach first, world ADD second). Without `is_diff=True`
  MoveIt treated the robot state as a replacement rather than a diff, leaving the block attached.
- **Full 7-step sequence confirmed working end-to-end.** Arm returns cleanly to home.
- Smooth trajectory (`data/trajectory_smooth_4001x7.txt`) committed — 99.8% velocity reduction.

---

## Known Issues / Debugging

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot not visible | Wrong launch file | Use `iiwa.launch.py` not `iiwa_planning.launch.py` |
| `No ContextLoader for planner_id` | pipeline/planner not set | `pipeline_id='ompl'`, `planner_id='RRTConnect'` |
| `Unable to sample valid states` | Goal out of reach | Remove orientation constraint; verify x=1.0 |
| scene_setup exits with no SUCCESS | spin_once too fast | Use `rclpy.spin()` + `raise SystemExit` in callback |
| Planning fails error 99999 | Goal unreachable or start state in collision | Restart everything; verify x=1.0 offset |
| GRIP fails ApplyPlanningScene=False | Object in tool0 frame | Use world frame for attached object |
| RETREAT fails error -16 | Pilz PTP fails post-PLACE | Use OMPL with joint goal |
| RETREAT fails error -4 (CONTROL_FAILED) | Arm at z=0.85, OMPL clips table | Cartesian pre-retreat ascent to z=0.95 (step 6) ✓ fixed |
| CARRY fails error -4 (CONTROL_FAILED) | OMPL start state stale after Cartesian execution | Subscribe to `/joint_states`; feed into `start_state` for every OMPL call ✓ fixed |
| Cartesian descent fails at 56% | Block in world scene triggers false collision at 4 cm gap | Remove block from world BEFORE descent; `avoid_collisions=True` now works ✓ fixed |
| Cartesian carry fails 55–63% | Orientation lock infeasible over 0.2 m horizontal | Use single OMPL goal for carry ✓ fixed |
| `jump_threshold=5.0` breaks Cartesian | Relative threshold at singularities | Use `jump_threshold=0.0` always ✓ fixed |
| Block stays on gripper after RELEASE | `robot_state.is_diff` not set; atomic detach confuses RViz | Two-stage release + `scene.robot_state.is_diff=True` ✓ fixed |
| `scipy` numpy dtype error | System scipy 1.8.0 incompatible with numpy 2.x | `pip3 install --user --upgrade scipy` ✓ fixed |
| `trajectory_smooth_4001x7.txt` missing | Not generated yet | `python3 scripts/generate_smooth_trajectory.py` |
| Step 1 spams infinitely | Ghost timer `.cancel` missing `()` | `_busy` flag + proper cancel ✓ fixed |

---

## Gazebo vs RViz — Block Motion

**RViz** shows MoveIt's planning scene (collision objects, ACOs). Updates when `ApplyPlanningScene` fires.
**Gazebo** is a separate physics simulator. The block won't move in Gazebo unless you explicitly:
- **Option A (current):** Delete the entity + respawn it at the new pose via `/delete_entity` and `/spawn_entity` services — this is what `_gazebo_respawn()` in `pick_and_place.py` does.
- **Option B (future):** Use the Gazebo link-attacher plugin so the block physically follows the gripper in real time during carry.

**How to confirm Option A is working:** Look for these lines in the terminal during step 5 (RELEASE):
```
Block re-spawned in Gazebo at place position!
```
If you only see `Block detached and placed in world scene` but NOT the re-spawn message, the Gazebo delete/spawn failed — check entity name or service availability.

**Current behaviour:** Block appears to teleport from pick position to place position in Gazebo at the moment of RELEASE (delete + respawn). It does not physically follow the arm during carry — that requires Option B.

---

## Next Steps

1. **Record demo video** — capture full 7-step pick-and-place side-by-side (Gazebo + RViz) for portfolio.
2. **Gazebo real-time carry (optional):** Integrate Gazebo link-attacher plugin so block physically follows arm during steps 3–5 instead of teleporting at release.
3. **Trajectory comparison demo** — run raw then smooth replay back-to-back; record both.
4. **Run analyze_trajectory.py** — generate velocity/acceleration comparison plots for report.
5. **Update technical report** — add Session 6 results: collision-fix, start-state fix, two-stage release, working full sequence.

---

## Package Entry Points

| Command | Script | Purpose |
|---------|--------|---------|
| `ros2 run iiwa_trajectory_demo pick_and_place` | `pick_and_place.py` | Full 7-step pick-and-place |
| `ros2 run iiwa_trajectory_demo scene_setup` | `scene_setup.py` | Add table+block to MoveIt |
| `ros2 run iiwa_trajectory_demo trajectory_replay` | `trajectory_replay.py` | Replay Nx7 trajectory file |
| `ros2 run iiwa_trajectory_demo move_to_target` | `move_to_target.py` | Move EE to target (quick test) |
