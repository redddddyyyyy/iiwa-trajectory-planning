#!/usr/bin/env python3
"""Add table and target block as MoveIt collision objects via /apply_planning_scene service."""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


def _make_box(obj_id, frame_id, x, y, z, dx, dy, dz):
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = obj_id
    obj.operation = CollisionObject.ADD

    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = [dx, dy, dz]

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    obj.primitives.append(shape)
    obj.primitive_poses.append(pose)
    return obj


class SceneSetupNode(Node):
    def __init__(self):
        super().__init__('scene_setup')
        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        self._client.wait_for_service()
        self.get_logger().info('Service ready. Applying scene...')
        self._apply()

    def _apply(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Robot base is at world x=1.0 — objects must be offset accordingly.
        # Table: 1.0 x 0.8 x 0.75 m, centre at (1.0, -0.6, 0.375) → top at z=0.75
        scene.world.collision_objects.append(
            _make_box('demo_table', 'world', 1.0, -0.6, 0.375, 1.0, 0.8, 0.75)
        )
        # Target block: 0.06 m cube, centre at (1.0, -0.3, 0.78)
        scene.world.collision_objects.append(
            _make_box('target_block', 'world', 1.0, -0.3, 0.78, 0.06, 0.06, 0.06)
        )

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._future = self._client.call_async(req)
        self._future.add_done_callback(self._on_response)

    def _on_response(self, future):
        if future.result().success:
            self.get_logger().info(
                'SUCCESS — demo_table and target_block added to MoveIt planning scene.'
            )
        else:
            self.get_logger().error('ApplyPlanningScene returned failure.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetupNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
