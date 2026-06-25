#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class HeadVioVisualizer(Node):
    """Republish OpenVINS head VIO pose into project topics and TF for RViz."""

    def __init__(self) -> None:
        super().__init__("head_vio_visualizer")
        self.declare_parameter("input_pose_topic", "/ov_msckf/poseimu")
        self.declare_parameter("input_type", "pose_with_covariance_stamped")
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("child_frame", "head_imu")
        self.declare_parameter("output_pose_topic", "/motion/head_pose")
        self.declare_parameter("output_path_topic", "/motion/head_path")
        self.declare_parameter("output_jsonl", "")
        self.declare_parameter("max_path_length", 5000)

        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.input_type = str(self.get_parameter("input_type").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.max_path_length = int(self.get_parameter("max_path_length").value)

        output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        output_path_topic = str(self.get_parameter("output_path_topic").value)
        output_jsonl = str(self.get_parameter("output_jsonl").value)
        self.output_jsonl_path = Path(output_jsonl).expanduser() if output_jsonl else None
        self.output_jsonl_file = None
        if self.output_jsonl_path is not None:
            self.output_jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_jsonl_file = self.output_jsonl_path.open("w", encoding="utf-8")

        self.pose_pub = self.create_publisher(PoseStamped, output_pose_topic, 10)
        self.path_pub = self.create_publisher(Path, output_path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        msg_type, callback = self._subscription_for_type(self.input_type)
        self.subscription = self.create_subscription(msg_type, self.input_pose_topic, callback, 10)

        self.get_logger().info(
            "head_vio_visualizer: %s (%s) -> %s, %s, TF %s -> %s"
            % (
                self.input_pose_topic,
                self.input_type,
                output_pose_topic,
                output_path_topic,
                self.fixed_frame,
                self.child_frame,
            )
        )

    def _subscription_for_type(self, input_type: str) -> tuple[type, Callable]:
        normalized = input_type.strip().lower()
        if normalized in {"pose_with_covariance_stamped", "posecov", "pose_with_covariance"}:
            return PoseWithCovarianceStamped, self._on_pose_with_covariance
        if normalized in {"pose_stamped", "pose"}:
            return PoseStamped, self._on_pose_stamped
        if normalized in {"odometry", "odom"}:
            return Odometry, self._on_odometry
        raise RuntimeError(
            "Unsupported input_type '%s'. Use pose_with_covariance_stamped, pose_stamped, or odometry."
            % input_type
        )

    def _on_pose_with_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        self._publish_pose(msg.header.stamp, msg.pose.pose)

    def _on_pose_stamped(self, msg: PoseStamped) -> None:
        self._publish_pose(msg.header.stamp, msg.pose)

    def _on_odometry(self, msg: Odometry) -> None:
        self._publish_pose(msg.header.stamp, msg.pose.pose)

    def _publish_pose(self, stamp, pose: Pose) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.fixed_frame
        pose_msg.pose = pose
        self.pose_pub.publish(pose_msg)

        self.path.header.stamp = stamp
        self.path.header.frame_id = self.fixed_frame
        self.path.poses.append(pose_msg)
        if self.max_path_length > 0 and len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length :]
        self.path_pub.publish(self.path)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.fixed_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self._write_jsonl_pose(stamp, pose)

    def _write_jsonl_pose(self, stamp, pose: Pose) -> None:
        if self.output_jsonl_file is None:
            return
        timestamp_unix_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        record = {
            "timestamp_unix_ns": timestamp_unix_ns,
            "timestamp_monotonic_ns": timestamp_unix_ns,
            "timestamp_source": "ros_header_stamp",
            "tracking_state": 1,
            "T_W_H": {
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "orientation_wxyz": [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            },
        }
        self.output_jsonl_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.output_jsonl_file.flush()

    def destroy_node(self) -> bool:
        if self.output_jsonl_file is not None:
            self.output_jsonl_file.close()
            self.output_jsonl_file = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HeadVioVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
