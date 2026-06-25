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

    input_pose_topic = LaunchConfiguration("input_pose_topic")
    input_type = LaunchConfiguration("input_type")
    fixed_frame = LaunchConfiguration("fixed_frame")
    child_frame = LaunchConfiguration("child_frame")
    output_pose_topic = LaunchConfiguration("output_pose_topic")
    output_path_topic = LaunchConfiguration("output_path_topic")
    output_jsonl = LaunchConfiguration("output_jsonl")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_pose_topic", default_value="/ov_msckf/poseimu"),
            DeclareLaunchArgument("input_type", default_value="pose_with_covariance_stamped"),
            DeclareLaunchArgument("fixed_frame", default_value="world"),
            DeclareLaunchArgument("child_frame", default_value="head_imu"),
            DeclareLaunchArgument("output_pose_topic", default_value="/motion/head_pose"),
            DeclareLaunchArgument("output_path_topic", default_value="/motion/head_path"),
            DeclareLaunchArgument("output_jsonl", default_value=""),
            Node(
                package="vimas_motion_bringup",
                executable="head_vio_visualizer.py",
                name="head_vio_visualizer",
                output="screen",
                parameters=[
                    {
                        "input_pose_topic": input_pose_topic,
                        "input_type": input_type,
                        "fixed_frame": fixed_frame,
                        "child_frame": child_frame,
                        "output_pose_topic": output_pose_topic,
                        "output_path_topic": output_path_topic,
                        "output_jsonl": output_jsonl,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_head_vio",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
