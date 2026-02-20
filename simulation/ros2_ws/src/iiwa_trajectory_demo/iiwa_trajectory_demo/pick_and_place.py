#!/usr/bin/env python3
"""
Pick and place demo for iiwa + target block.

Sequence:
  1. Move to APPROACH  (above block)
  2. Move to GRASP     (at block level)
  3. GRIP              — attach block to tool0 in MoveIt scene
  4. LIFT              — rise with block attached
  5. Move to PLACE     (different spot on table)
  6. RELEASE           — detach block + re-spawn in Gazebo
  7. RETREAT           — move arm clear

Robot base is at world x=1.0.
Block at world (1.0, -0.3, 0.78), table top at z=0.75.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, JointConstraint,
    BoundingVolume, PlanningScene, AttachedCollisionObject, CollisionObject,
)
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

# ── Waypoints (world frame, x=1.0 because robot base is at world x=1.0) ─────
APPROACH = (1.0, -0.3, 0.92)   # above the block
GRASP    = (1.0, -0.3, 0.85)   # just above block top (0.81 m)
LIFT     = (1.0, -0.3, 0.95)   # lift clearly above table
PLACE    = (1.0, -0.5, 0.90)   # different spot on same table, clearly separate

# Gazebo re-spawn at table surface height so it visually sits on the table
PLACE_GAZEBO = (1.0, -0.5, 0.78)

# Home joint state (from SRDF "ready" state) — used for clean RETREAT
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
    'Move to APPROACH (above block)',       # 0
    'Move to GRASP (at block)',             # 1
    'GRIP — attach block to tool0',         # 2
    'LIFT — move up with block',            # 3
    'Move to PLACE position',              # 4
    'RELEASE — detach + re-spawn in Gazebo',# 5
    'RETREAT — move arm clear',            # 6
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

        self._move_client   = ActionClient(self, MoveGroup, '/move_action')
        self._scene_client  = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn_client  = self.create_client(SpawnEntity, '/spawn_entity')

        self.get_logger().info('Waiting for servers...')
        self._move_client.wait_for_server()
        self._scene_client.wait_for_service()
        self._delete_client.wait_for_service()
        self._spawn_client.wait_for_service()

        self.get_logger().info('All servers ready. Starting in 1 s...')
        self._step = 0
        self._busy = False                          # blocks re-entry
        self._timer = self.create_timer(1.0, self._tick)

    # ── State machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self._busy:
            return                                  # wait for current step
        self._timer.cancel()                        # stop periodic firing

        s = self._step
        if s >= len(STEPS):
            self.get_logger().info('Pick-and-place sequence COMPLETE!')
            return

        self.get_logger().info(f'--- Step {s + 1}/{len(STEPS)}: {STEPS[s]} ---')
        self._busy = True

        if s == 0:
            self._move_to(*APPROACH)
        elif s == 1:
            self._move_to(*GRASP)
        elif s == 2:
            self._grip()
        elif s == 3:
            self._move_to(*LIFT)
        elif s == 4:
            self._move_to(*PLACE)
        elif s == 5:
            self._release()
        elif s == 6:
            self._retreat_home()

    def _advance(self, success=True):
        """Called when a step finishes. Schedules the next step."""
        self._busy = False
        if not success:
            self.get_logger().error(f'Step {self._step + 1} FAILED — stopping.')
            return
        self._step += 1
        if self._step >= len(STEPS):
            self.get_logger().info('Pick-and-place sequence COMPLETE!')
            return
        # Small pause between steps, then fire _tick once
        self._timer = self.create_timer(1.5, self._tick)

    # ── Motion ────────────────────────────────────────────────────────────────

    def _move_to(self, x, y, z):
        req = MotionPlanRequest()
        req.group_name = 'iiwa_arm'
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -0.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = 0.0
        req.workspace_parameters.max_corner.x = 2.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 2.0

        # Goal constraint
        c = Constraints()
        c.position_constraints.append(_pos_constraint(x, y, z))
        req.goal_constraints.append(c)

        # Path constraint: keep tool0 above table (z > 0.80) at ALL times.
        # This prevents the arm from sweeping under/through the table during transit.
        path_pos = PositionConstraint()
        path_pos.header.frame_id = 'world'
        path_pos.link_name = 'tool0'
        path_pos.weight = 1.0

        # A tall box from z=0.80 upward covers the safe operating zone
        path_box = SolidPrimitive()
        path_box.type = SolidPrimitive.BOX
        path_box.dimensions = [4.0, 4.0, 2.0]   # wide XY, 2 m tall

        path_tp = Pose()
        path_tp.position.x = 1.0
        path_tp.position.y = -0.3
        path_tp.position.z = 1.80   # centre of [0.80, 2.80]
        path_tp.orientation.w = 1.0

        path_bvol = BoundingVolume()
        path_bvol.primitives.append(path_box)
        path_bvol.primitive_poses.append(path_tp)
        path_pos.constraint_region = path_bvol

        req.path_constraints.position_constraints.append(path_pos)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
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
        if ok:
            self.get_logger().info('Motion succeeded.')
        else:
            self.get_logger().error(f'Motion failed (error_code={code}).')
        self._advance(ok)

    def _retreat_home(self):
        """Move to the SRDF 'ready' joint state using OMPL (flexible path around obstacles)."""
        req = MotionPlanRequest()
        req.group_name = 'iiwa_arm'
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4

        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -0.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = 0.0
        req.workspace_parameters.max_corner.x = 2.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 2.0

        c = Constraints()
        for name, value in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.05   # looser — lets OMPL find goal faster
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        fut = self._move_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_accepted)

    # ── Grip ─────────────────────────────────────────────────────────────────

    def _grip(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Attach block to tool0 using WORLD frame position.
        # (tool0 is rotated -90° around Y, so tool0-frame offsets are misleading.)
        aco = AttachedCollisionObject()
        aco.link_name = 'tool0'
        # Allow contact with end-effector links so collision check doesn't fail
        aco.touch_links = ['tool0', 'link_7', 'link_6', 'link_5']

        obj = CollisionObject()
        obj.header.frame_id = 'world'   # use world frame — safe and unambiguous
        obj.id = 'target_block'
        obj.operation = CollisionObject.ADD

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.06, 0.06, 0.06]

        pose = Pose()
        pose.position.x = 1.0    # block's world position
        pose.position.y = -0.3
        pose.position.z = 0.78   # block centre
        pose.orientation.w = 1.0

        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        aco.object = obj
        scene.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(lambda f: self._on_scene_done(f, 'Block GRIPPED (attached to tool0).'))

    # ── Release ───────────────────────────────────────────────────────────────

    def _release(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Detach from robot
        aco = AttachedCollisionObject()
        aco.link_name = 'tool0'
        aco.object.id = 'target_block'
        aco.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(aco)

        # Place back in world at new position
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = 'target_block'
        obj.operation = CollisionObject.ADD

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.06, 0.06, 0.06]

        pose = Pose()
        pose.position.x = float(PLACE_GAZEBO[0])
        pose.position.y = float(PLACE_GAZEBO[1])
        pose.position.z = float(PLACE_GAZEBO[2])
        pose.orientation.w = 1.0

        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._scene_client.call_async(req)
        fut.add_done_callback(self._on_release_scene_done)

    def _on_release_scene_done(self, future):
        if future.result().success:
            self.get_logger().info('Block detached in MoveIt. Updating Gazebo...')
            self._gazebo_respawn()
        else:
            self.get_logger().error('Detach from MoveIt failed.')
            self._advance(False)

    def _gazebo_respawn(self):
        req = DeleteEntity.Request()
        req.name = 'target_block'
        fut = self._delete_client.call_async(req)
        fut.add_done_callback(self._on_gazebo_deleted)

    def _on_gazebo_deleted(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().warn(f'Gazebo delete: {result.status_message}')
        req = SpawnEntity.Request()
        req.name = 'target_block'
        req.xml = TARGET_BLOCK_SDF
        req.initial_pose.position.x = float(PLACE_GAZEBO[0])
        req.initial_pose.position.y = float(PLACE_GAZEBO[1])
        req.initial_pose.position.z = float(PLACE_GAZEBO[2])
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

    # ── Shared scene callback ─────────────────────────────────────────────────

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
