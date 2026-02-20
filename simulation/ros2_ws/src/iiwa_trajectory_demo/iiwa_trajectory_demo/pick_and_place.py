#!/usr/bin/env python3
"""
Pick and place demo for iiwa + target block.

7-step hybrid Cartesian + OMPL sequence:
  1. OMPL    APPROACH  (1.0, -0.3, 0.92) — free motion from home to above block
  2. Cartesian descent → GRASP (1.0, -0.3, 0.85) — straight line down, orientation locked from TF
  3. GRIP    attach block to tool0 (two-stage: world REMOVE then ACO attach)
  4. OMPL    PLACE (1.0, -0.5, 0.90) — single clean 0.2 m carry move
  5. RELEASE detach ACO + world ADD + Gazebo delete/respawn at (1.0, -0.5, 0.78)
  6. Cartesian ascent to z=0.95 — clears table so OMPL can plan freely
  7. OMPL    RETREAT to HOME joints (SRDF ready state)

Design decisions:
  - Cartesian only for short vertical segments (steps 2, 6) — orientation lock reliable for vertical
  - OMPL for all horizontal/free-space moves (steps 1, 4, 7)
  - Cartesian carry (GRASP→PLACE) failed 55–63%: orientation lock infeasible over 0.2 m horizontal span
  - jump_threshold=0.0 on all Cartesian paths (5.0 falsely flags motion near singularities)
  - avoid_collisions=False on Cartesian paths (both operate in proven clear-air zones)

Robot base is at world x=1.0.
Block at world (1.0, -0.3, 0.78), table top at z=0.75.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, JointConstraint,
    BoundingVolume, PlanningScene, AttachedCollisionObject, CollisionObject,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

# ── Waypoints (world frame, x=1.0 because robot base is at world x=1.0) ─────
APPROACH = (1.0, -0.3, 0.92)   # safe pose above the block
GRASP    = (1.0, -0.3, 0.85)   # descend to just above block top (0.81 m)
PLACE    = (1.0, -0.5, 0.90)   # OMPL carry destination — clear of table, easy retreat

# Gazebo re-spawn position (block centre on table surface)
PLACE_GAZEBO = (1.0, -0.5, 0.78)

# Home joint state (SRDF "ready" state) — used for RETREAT
HOME_JOINTS = {
    'joint_a1': 0.0,
    'joint_a2': -0.785,
    'joint_a3': 0.0,
    'joint_a4': -2.356,
    'joint_a5': 0.0,
    'joint_a6':  1.571,
    'joint_a7':  0.785,
}

TARGET_BLOCK_SDF = """\
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="target_block">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
        <material><ambient>0.9 0.2 0.2 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>"""

STEPS = [
    'OMPL free-motion to APPROACH (above block)',       # 0
    'Cartesian descent  APPROACH → GRASP (straight ↓)', # 1
    'GRIP — attach block to tool0',                     # 2
    'OMPL carry  GRASP → PLACE  (one clean move)',       # 3
    'RELEASE — detach + re-spawn in Gazebo',            # 4
    'Cartesian ascent to safe height',                  # 5
    'RETREAT — return to home joints',                  # 6
]


def _pos_constraint(x, y, z, tol=0.03):
    pos = PositionConstraint()
    pos.header.frame_id = 'world'
    pos.link_name = 'tool0'
    pos.weight = 1.0
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [tol, tol, tol]
    tp = Pose()
    tp.position.x = x
    tp.position.y = y
    tp.position.z = z
    tp.orientation.w = 1.0
    bvol = BoundingVolume()
    bvol.primitives.append(box)
    bvol.primitive_poses.append(tp)
    pos.constraint_region = bvol
    return pos


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # ── Clients ───────────────────────────────────────────────────────────
        self._move_client      = ActionClient(self, MoveGroup,          '/move_action')
        self._exec_client      = ActionClient(self, ExecuteTrajectory,  '/execute_trajectory')
        self._scene_client     = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._cartesian_client = self.create_client(GetCartesianPath,   '/compute_cartesian_path')
        self._delete_client    = self.create_client(DeleteEntity,       '/delete_entity')
        self._spawn_client     = self.create_client(SpawnEntity,        '/spawn_entity')

        # ── TF listener — used to read current EE pose for Cartesian paths ───
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Joint-state subscriber — actual controller positions for OMPL start ─
        # After Cartesian execution MoveIt's internal model can drift from the
        # controller's real joint positions.  Feeding the live /joint_states into
        # start_state ensures the planned trajectory starts exactly where the arm
        # actually is, preventing "start deviates > 0.01" execution rejections.
        self._joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        self.get_logger().info('Waiting for servers...')
        self._move_client.wait_for_server()
        self._exec_client.wait_for_server()
        self._scene_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self._delete_client.wait_for_service()
        self._spawn_client.wait_for_service()

        self.get_logger().info('All servers ready. Starting in 1 s...')
        self._step = 0
        self._busy = False
        self._pending_cartesian_log = ''
        self._timer = self.create_timer(1.0, self._tick)

    # ── State machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self._busy:
            return
        self._timer.cancel()

        s = self._step
        if s >= len(STEPS):
            self.get_logger().info('Pick-and-place sequence COMPLETE!')
            return

        self.get_logger().info(f'--- Step {s + 1}/{len(STEPS)}: {STEPS[s]} ---')
        self._busy = True

        if   s == 0: self._move_to(*APPROACH)
        elif s == 1: self._cartesian_descent()
        elif s == 2: self._grip()
        elif s == 3: self._move_to(*PLACE)
        elif s == 4: self._release()
        elif s == 5: self._cartesian_pre_retreat()
        elif s == 6: self._retreat_home()

    def _advance(self, success=True):
        self._busy = False
        if not success:
            self.get_logger().error(f'Step {self._step + 1} FAILED — stopping.')
            return
        self._step += 1
        if self._step >= len(STEPS):
            self.get_logger().info('Pick-and-place sequence COMPLETE!')
            return
        self._timer = self.create_timer(1.5, self._tick)

    def _on_joint_states(self, msg):
        self._joint_state = msg

    def _set_start_state(self, req):
        """Populate start_state from live /joint_states so OMPL plans from the
        actual controller position, not MoveIt's potentially stale cached model."""
        if self._joint_state is not None:
            req.start_state.joint_state = self._joint_state
        else:
            req.start_state.is_diff = True  # fallback if /joint_states not yet received

    # ── OMPL free-motion ──────────────────────────────────────────────────────

    def _move_to(self, x, y, z):
        req = MotionPlanRequest()
        req.group_name = 'iiwa_arm'
        req.pipeline_id = 'ompl'
        req.planner_id  = 'RRTConnect'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor     = 0.5
        req.max_acceleration_scaling_factor = 0.5
        self._set_start_state(req)
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -0.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z =  0.0
        req.workspace_parameters.max_corner.x =  2.5
        req.workspace_parameters.max_corner.y =  1.5
        req.workspace_parameters.max_corner.z =  2.0
        c = Constraints()
        c.position_constraints.append(_pos_constraint(x, y, z))
        req.goal_constraints.append(c)
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only      = False
        goal.planning_options.replan         = True
        goal.planning_options.replan_attempts = 3
        fut = self._move_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Motion goal rejected by MoveGroup.')
            self._advance(False)
            return
        handle.get_result_async().add_done_callback(self._on_move_result)

    def _on_move_result(self, future):
        code = future.result().result.error_code.val
        ok = (code == 1)
        self.get_logger().info('Motion succeeded.' if ok else
                               f'Motion failed (error_code={code}).')
        self._advance(ok)

    def _cartesian_pre_retreat(self):
        """Lift straight up to a safe height before OMPL plans back to home.

        After PLACE the arm sits at z≈0.85, only 10 cm above the table.
        OMPL paths from that height tend to clip the table and get rejected
        by the controller (error -4 CONTROL_FAILED).  Rising to z=0.95 first
        puts the arm in clear air so OMPL can plan freely.
        """
        cur = self._get_ee_pose()
        if cur is None:
            self.get_logger().warn('TF unavailable for pre-retreat ascent — skipping to OMPL.')
            self._advance(True)   # skip this step, let OMPL try anyway
            return

        safe = Pose()
        safe.position.x  = cur.position.x
        safe.position.y  = cur.position.y
        safe.position.z  = 0.95        # well above table (table top = 0.75)
        safe.orientation = cur.orientation

        self._pending_cartesian_log = 'Pre-retreat ascent complete — arm clear of table.'
        self._send_cartesian_path([safe], avoid_collisions=True)

    def _retreat_home(self):
        req = MotionPlanRequest()
        req.group_name = 'iiwa_arm'
        req.pipeline_id = 'ompl'
        req.planner_id  = 'RRTConnect'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor     = 0.4
        req.max_acceleration_scaling_factor = 0.4
        self._set_start_state(req)
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -0.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z =  0.0
        req.workspace_parameters.max_corner.x =  2.5
        req.workspace_parameters.max_corner.y =  1.5
        req.workspace_parameters.max_corner.z =  2.0
        c = Constraints()
        for name, value in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name     = name
            jc.position       = value
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only      = False
        goal.planning_options.replan         = True
        goal.planning_options.replan_attempts = 3
        fut = self._move_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_accepted)

    # ── Cartesian path helpers ────────────────────────────────────────────────

    def _get_ee_pose(self):
        """Return current tool0 pose in world frame, or None on TF failure."""
        try:
            t = self._tf_buffer.lookup_transform(
                'world', 'tool0', rclpy.time.Time())
            p = Pose()
            p.position.x  = t.transform.translation.x
            p.position.y  = t.transform.translation.y
            p.position.z  = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception as e:
            self.get_logger().error(f'TF lookup world←tool0 failed: {e}')
            return None

    def _cartesian_descent(self):
        """Remove block from world scene, then descend straight to GRASP.

        Removing the block first lets us use avoid_collisions=True (correct
        behaviour) without the 4 cm gap triggering a false collision.  The block
        is re-added to the scene as an ACO attached to tool0 in _grip().
        """
        scene = PlanningScene()
        scene.is_diff = True
        obj_rm = CollisionObject()
        obj_rm.header.frame_id = 'world'
        obj_rm.id        = 'target_block'
        obj_rm.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(obj_rm)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(self._on_block_removed_for_descent)

    def _on_block_removed_for_descent(self, _future):
        # Ignore service result — block may already be absent (scene_setup not run).
        cur = self._get_ee_pose()
        if cur is None:
            self._advance(False)
            return
        grasp = Pose()
        grasp.position.x  = GRASP[0]
        grasp.position.y  = GRASP[1]
        grasp.position.z  = GRASP[2]
        grasp.orientation = cur.orientation
        self._pending_cartesian_log = 'Cartesian descent complete — at GRASP position.'
        self._send_cartesian_path([grasp], avoid_collisions=True)

    def _send_cartesian_path(self, waypoints, avoid_collisions=True):
        req = GetCartesianPath.Request()
        req.header.frame_id  = 'world'
        req.group_name       = 'iiwa_arm'
        req.link_name        = 'tool0'
        req.waypoints        = waypoints
        req.max_step         = 0.01     # 1 cm interpolation step
        req.jump_threshold   = 0.0     # 0.0 = disabled; 5.0 caused false failures
                                        # because it's relative (N× avg), not absolute
        req.avoid_collisions = avoid_collisions
        fut = self._cartesian_client.call_async(req)
        fut.add_done_callback(self._on_cartesian_computed)

    def _on_cartesian_computed(self, future):
        resp = future.result()
        pct  = resp.fraction * 100
        if resp.fraction < 0.9:
            self.get_logger().error(
                f'Cartesian path only {pct:.0f}% achieved — aborting step.')
            self._advance(False)
            return
        if resp.fraction < 1.0:
            self.get_logger().warn(f'Cartesian path {pct:.0f}% achieved (partial, continuing).')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        fut = self._exec_client.send_goal_async(goal)
        fut.add_done_callback(self._on_exec_accepted)

    def _on_exec_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Trajectory execution rejected.')
            self._advance(False)
            return
        handle.get_result_async().add_done_callback(self._on_exec_result)

    def _on_exec_result(self, future):
        code = future.result().result.error_code.val
        ok   = (code == 1)
        if ok:
            self.get_logger().info(self._pending_cartesian_log)
        else:
            self.get_logger().error(f'Cartesian execution failed (error_code={code}).')
        self._advance(ok)

    # ── Grip ─────────────────────────────────────────────────────────────────

    def _grip(self):
        # The world copy of target_block was already removed in _cartesian_descent().
        # Just attach it as an ACO to tool0 at its known world position.
        scene = PlanningScene()
        scene.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name  = 'tool0'
        aco.touch_links = ['tool0', 'link_7', 'link_6', 'link_5']
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id        = 'target_block'
        obj.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type       = SolidPrimitive.BOX
        shape.dimensions = [0.06, 0.06, 0.06]
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = -0.3
        pose.position.z = 0.78
        pose.orientation.w = 1.0
        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        aco.object = obj
        scene.robot_state.attached_collision_objects.append(aco)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(
            lambda f: self._on_scene_done(f, 'Block GRIPPED (attached to tool0).'))

    # ── Release ───────────────────────────────────────────────────────────────

    def _release(self):
        # Stage A: detach ACO only.
        # robot_state.is_diff=True tells MoveIt to treat attached_collision_objects
        # as a diff (remove this entry) rather than "this IS the full attached list".
        # Without is_diff=True the block stays visually attached in RViz.
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        aco = AttachedCollisionObject()
        aco.link_name = 'tool0'
        obj_detach = CollisionObject()
        obj_detach.id        = 'target_block'
        obj_detach.operation = CollisionObject.REMOVE
        aco.object = obj_detach
        scene.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(self._on_detach_done)

    def _on_detach_done(self, future):
        if not future.result().success:
            self.get_logger().error('ACO detach failed.')
            self._advance(False)
            return
        self.get_logger().info('Block detached from tool0.')

        # Stage B: place block in world at PLACE_GAZEBO.
        scene = PlanningScene()
        scene.is_diff = True

        obj_place = CollisionObject()
        obj_place.header.frame_id = 'world'
        obj_place.id        = 'target_block'
        obj_place.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type       = SolidPrimitive.BOX
        shape.dimensions = [0.06, 0.06, 0.06]
        pose = Pose()
        pose.position.x    = float(PLACE_GAZEBO[0])
        pose.position.y    = float(PLACE_GAZEBO[1])
        pose.position.z    = float(PLACE_GAZEBO[2])
        pose.orientation.w = 1.0
        obj_place.primitives.append(shape)
        obj_place.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj_place)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(self._on_release_done)

    def _on_release_done(self, future):
        if not future.result().success:
            self.get_logger().error('World place failed.')
            self._advance(False)
            return
        self.get_logger().info('Block placed in world scene. Updating Gazebo...')
        self._gazebo_respawn()

    # ── Gazebo respawn ────────────────────────────────────────────────────────

    def _gazebo_respawn(self):
        req = DeleteEntity.Request()
        req.name = 'target_block'
        fut = self._delete_client.call_async(req)
        fut.add_done_callback(self._on_gazebo_deleted)

    def _on_gazebo_deleted(self, future):
        if not future.result().success:
            self.get_logger().warn(
                f'Gazebo delete: {future.result().status_message}')
        req = SpawnEntity.Request()
        req.name = 'target_block'
        req.xml  = TARGET_BLOCK_SDF
        req.initial_pose.position.x    = float(PLACE_GAZEBO[0])
        req.initial_pose.position.y    = float(PLACE_GAZEBO[1])
        req.initial_pose.position.z    = float(PLACE_GAZEBO[2])
        req.initial_pose.orientation.w = 1.0
        fut = self._spawn_client.call_async(req)
        fut.add_done_callback(self._on_gazebo_spawned)

    def _on_gazebo_spawned(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info('Block re-spawned in Gazebo at place position!')
        else:
            self.get_logger().warn(f'Spawn: {result.status_message}')
        self._advance(True)

    # ── Shared planning-scene callback ────────────────────────────────────────

    def _on_scene_done(self, future, success_msg):
        ok = future.result().success
        if ok:
            self.get_logger().info(success_msg)
        else:
            self.get_logger().error('Planning scene update failed.')
        self._advance(ok)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
