from __future__ import annotations

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("vimas_motion_bringup"), "rviz", "head_vio.rviz"]
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_motion_live",
                arguments=["-d", rviz_config],
                output="screen",
            )
        ]
    )
