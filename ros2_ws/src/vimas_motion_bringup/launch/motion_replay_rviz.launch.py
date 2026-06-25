from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("vimas_motion_bringup"), "rviz", "head_vio.rviz"]
    )

    motion_jsonl = LaunchConfiguration("motion_jsonl")
    fixed_frame = LaunchConfiguration("fixed_frame")
    rate_hz = LaunchConfiguration("rate_hz")
    loop = LaunchConfiguration("loop")

    return LaunchDescription(
        [
            DeclareLaunchArgument("motion_jsonl", default_value=""),
            DeclareLaunchArgument("fixed_frame", default_value="world"),
            DeclareLaunchArgument("rate_hz", default_value="30.0"),
            DeclareLaunchArgument("loop", default_value="false"),
            Node(
                package="vimas_motion_bringup",
                executable="motion_replay_visualizer.py",
                name="motion_replay_visualizer",
                output="screen",
                parameters=[
                    {
                        "motion_jsonl": motion_jsonl,
                        "fixed_frame": fixed_frame,
                        "rate_hz": rate_hz,
                        "loop": loop,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_motion_replay",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
