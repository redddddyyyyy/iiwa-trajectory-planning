from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    trajectory_file_arg = DeclareLaunchArgument(
        "trajectory_file",
        default_value="",
        description="Absolute path to Nx7 trajectory file. Empty = package default.",
    )
    controller_topic_arg = DeclareLaunchArgument(
        "controller_topic",
        default_value="/joint_trajectory_controller/joint_trajectory",
        description="JointTrajectory command topic.",
    )
    time_scale_arg = DeclareLaunchArgument(
        "time_scale",
        default_value="1.0",
        description="Playback speed multiplier (>1 faster, <1 slower).",
    )

    replay_node = Node(
        package="iiwa_trajectory_demo",
        executable="trajectory_replay",
        output="screen",
        parameters=[
            {
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "controller_topic": LaunchConfiguration("controller_topic"),
                "time_scale": LaunchConfiguration("time_scale"),
            }
        ],
    )

    return LaunchDescription(
        [
            trajectory_file_arg,
            controller_topic_arg,
            time_scale_arg,
            replay_node,
        ]
    )
