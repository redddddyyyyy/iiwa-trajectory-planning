#!/usr/bin/env python3
"""Move the iiwa end-effector (tool0) to above the target block using MoveIt2."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    BoundingVolume,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


# Robot base is at world x=1.0. Target block is at world (1.0, -0.3, 0.78).
# Goal: tool0 hovers ~7 cm above the block top (z=0.81 → goal z=0.88).
TARGET_X = 1.0
TARGET_Y = -0.3
TARGET_Z = 0.88


class MoveToTargetNode(Node):
    def __init__(self):
        super().__init__('move_to_target')
        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info('Waiting for /move_action server...')
        self._client.wait_for_server()
        self.get_logger().info('Connected. Sending motion goal...')
        self._send_goal()

    def _send_goal(self):
        request = MotionPlanRequest()
        request.group_name = 'iiwa_arm'
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnect'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 15.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        # Reachable workspace bounds
        request.workspace_parameters.header.frame_id = 'world'
        request.workspace_parameters.min_corner.x = -1.5
        request.workspace_parameters.min_corner.y = -1.5
        request.workspace_parameters.min_corner.z = 0.0
        request.workspace_parameters.max_corner.x = 1.5
        request.workspace_parameters.max_corner.y = 1.5
        request.workspace_parameters.max_corner.z = 2.0

        # --- Position constraint only (no orientation — gives IK maximum freedom) ---
        pos = PositionConstraint()
        pos.header.frame_id = 'world'
        pos.link_name = 'tool0'
        pos.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]   # 2 cm tolerance box

        tol_pose = Pose()
        tol_pose.position.x = TARGET_X
        tol_pose.position.y = TARGET_Y
        tol_pose.position.z = TARGET_Z
        tol_pose.orientation.w = 1.0

        bvol = BoundingVolume()
        bvol.primitives.append(box)
        bvol.primitive_poses.append(tol_pose)
        pos.constraint_region = bvol

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos)
        request.goal_constraints.append(goal_constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False   # plan AND execute
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        future = self._client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(f'Planning state: {feedback_msg.feedback.state}')

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup!')
            return
        self.get_logger().info('Goal accepted — planning and executing...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        code = result.error_code.val
        if code == 1:
            self.get_logger().info('SUCCESS — end-effector reached above target block!')
        else:
            self.get_logger().error(
                f'Motion failed (error code {code}). '
                'Try re-running or check for collisions / IK limits.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MoveToTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
