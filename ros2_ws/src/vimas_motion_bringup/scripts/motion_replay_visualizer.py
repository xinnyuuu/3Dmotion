#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MotionReplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("motion_replay_visualizer")
        self.declare_parameter("motion_jsonl", "")
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("head_frame", "head")
        self.declare_parameter("wrist_frame", "wrist")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("max_path_length", 5000)

        self.motion_jsonl = Path(str(self.get_parameter("motion_jsonl").value)).expanduser()
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.head_frame = str(self.get_parameter("head_frame").value)
        self.wrist_frame = str(self.get_parameter("wrist_frame").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.max_path_length = int(self.get_parameter("max_path_length").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        if not self.motion_jsonl.exists():
            raise RuntimeError(f"motion_jsonl does not exist: {self.motion_jsonl}")
        self.frames = [json.loads(line) for line in self.motion_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not self.frames:
            raise RuntimeError(f"motion_jsonl is empty: {self.motion_jsonl}")

        self.head_pose_pub = self.create_publisher(PoseStamped, "/motion/head_pose", 10)
        self.wrist_pose_pub = self.create_publisher(PoseStamped, "/motion/wrist_pose", 10)
        self.head_path_pub = self.create_publisher(PathMsg, "/motion/head_path", 10)
        self.wrist_path_pub = self.create_publisher(PathMsg, "/motion/wrist_path", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.head_path = PathMsg()
        self.head_path.header.frame_id = self.fixed_frame
        self.wrist_path = PathMsg()
        self.wrist_path.header.frame_id = self.fixed_frame
        self.index = 0
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-6), self._tick)
        self.get_logger().info(f"Replaying {len(self.frames)} motion frames from {self.motion_jsonl}")

    def _tick(self) -> None:
        if self.index >= len(self.frames):
            if not self.loop:
                return
            self.index = 0
            self.head_path.poses.clear()
            self.wrist_path.poses.clear()

        frame = self.frames[self.index]
        self.index += 1
        stamp = self.get_clock().now().to_msg()
        head_pose = self._pose_msg(frame["head"], stamp)
        wrist_pose = self._pose_msg(frame["wrist"], stamp)
        self.head_pose_pub.publish(head_pose)
        self.wrist_pose_pub.publish(wrist_pose)
        self._append_path(self.head_path, head_pose)
        self._append_path(self.wrist_path, wrist_pose)
        self.head_path_pub.publish(self.head_path)
        self.wrist_path_pub.publish(self.wrist_path)
        self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.head_frame, frame["head"], stamp))
        relative = frame.get("relative", {}).get("T_H_B")
        if relative:
            self.tf_broadcaster.sendTransform(self._transform_msg(self.head_frame, self.wrist_frame, relative, stamp))
        else:
            self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.wrist_frame, frame["wrist"], stamp))

    def _pose_msg(self, state: dict, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fixed_frame
        position = state["position"]
        quat = state["orientation_wxyz"]
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        return msg

    def _transform_msg(self, frame_id: str, child_frame_id: str, state: dict, stamp) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        position = state["position"]
        quat = state["orientation_wxyz"]
        msg.transform.translation.x = float(position[0])
        msg.transform.translation.y = float(position[1])
        msg.transform.translation.z = float(position[2])
        msg.transform.rotation.w = float(quat[0])
        msg.transform.rotation.x = float(quat[1])
        msg.transform.rotation.y = float(quat[2])
        msg.transform.rotation.z = float(quat[3])
        return msg

    def _append_path(self, path: PathMsg, pose: PoseStamped) -> None:
        path.header.stamp = pose.header.stamp
        path.header.frame_id = self.fixed_frame
        path.poses.append(pose)
        if self.max_path_length > 0 and len(path.poses) > self.max_path_length:
            path.poses = path.poses[-self.max_path_length :]


def main() -> None:
    rclpy.init()
    node = MotionReplayVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
