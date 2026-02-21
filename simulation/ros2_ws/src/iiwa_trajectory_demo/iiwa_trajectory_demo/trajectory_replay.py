#!/usr/bin/env python3
"""Replay an Nx7 joint trajectory file to a ROS2 JointTrajectory controller."""

from pathlib import Path
from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _load_trajectory(path: Path, expected_cols: int = 7) -> List[List[float]]:
    rows: List[List[float]] = []
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split()
        if len(parts) != expected_cols:
            raise ValueError(
                f"{path}:{lineno} expected {expected_cols} values, got {len(parts)}"
            )
        rows.append([float(x) for x in parts])

    if not rows:
        raise ValueError(f"Trajectory file {path} has no data")
    return rows


def _default_trajectory_path() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "data" / "trajectory_smooth_4001x7.txt"
        if candidate.exists():
            return candidate
    return Path("data/trajectory_smooth_4001x7.txt")


class TrajectoryReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("iiwa_trajectory_replay")

        default_traj = _default_trajectory_path()

        self.declare_parameter("trajectory_file", str(default_traj))
        self.declare_parameter(
            "joint_names",
            ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"],
        )
        self.declare_parameter("controller_topic", "/iiwa_arm_controller/joint_trajectory")
        self.declare_parameter("sample_dt", 0.005)
        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("start_delay_sec", 2.0)

        trajectory_file_param = str(self.get_parameter("trajectory_file").value).strip()
        trajectory_file = Path(trajectory_file_param) if trajectory_file_param else default_traj
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.controller_topic = str(self.get_parameter("controller_topic").value)
        self.sample_dt = float(self.get_parameter("sample_dt").value)
        self.time_scale = float(self.get_parameter("time_scale").value)
        start_delay_sec = float(self.get_parameter("start_delay_sec").value)

        if len(self.joint_names) != 7:
            raise ValueError("joint_names must contain exactly 7 entries")
        if self.sample_dt <= 0.0:
            raise ValueError("sample_dt must be > 0")
        if self.time_scale <= 0.0:
            raise ValueError("time_scale must be > 0")

        self.pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.rows = _load_trajectory(trajectory_file)

        self.get_logger().info(
            f"Loaded {len(self.rows)} trajectory points from {trajectory_file}"
        )
        self.get_logger().info(
            f"Publishing once to {self.controller_topic} after {start_delay_sec:.2f}s"
        )

        self.timer = self.create_timer(start_delay_sec, self._publish_once)
        self._sent = False

    def _publish_once(self) -> None:
        if self._sent:
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        dt = self.sample_dt / self.time_scale
        elapsed = 0.0
        for row in self.rows:
            point = JointTrajectoryPoint()
            point.positions = row
            elapsed += dt
            sec = int(elapsed)
            nanosec = int(round((elapsed - sec) * 1e9))
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
            point.time_from_start = Duration(
                sec=sec, nanosec=nanosec
            )
            msg.points.append(point)

        self.pub.publish(msg)
        self._sent = True
        self.timer.cancel()
        self.get_logger().info(
            f"Published trajectory with {len(msg.points)} points. Node will stay alive."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
