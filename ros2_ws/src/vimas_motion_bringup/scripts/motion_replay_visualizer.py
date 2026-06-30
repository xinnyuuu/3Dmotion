#!/usr/bin/python3
from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter("publish_camera_images", True)
        self.declare_parameter("camera_frames_jsonl", "")
        self.declare_parameter("camera_image_max_width", 480)
        self.declare_parameter("camera_rate_hz", 5.0)

        self.motion_jsonl = Path(str(self.get_parameter("motion_jsonl").value)).expanduser()
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.head_frame = str(self.get_parameter("head_frame").value)
        self.wrist_frame = str(self.get_parameter("wrist_frame").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.max_path_length = int(self.get_parameter("max_path_length").value)
        self.publish_camera_images = bool(self.get_parameter("publish_camera_images").value)
        camera_frames_jsonl_param = str(self.get_parameter("camera_frames_jsonl").value).strip()
        self.camera_frames_jsonl = Path(camera_frames_jsonl_param).expanduser() if camera_frames_jsonl_param else None
        self.camera_image_max_width = int(self.get_parameter("camera_image_max_width").value)
        self.camera_rate_hz = float(self.get_parameter("camera_rate_hz").value)
        self.camera_publish_period_s = 1.0 / self.camera_rate_hz if self.camera_rate_hz > 0.0 else 0.0
        self.last_camera_publish_monotonic = 0.0
        self.rate_hz = max(float(self.get_parameter("rate_hz").value), 1e-6)

        if not self.motion_jsonl.exists():
            raise RuntimeError(f"motion_jsonl does not exist: {self.motion_jsonl}")
        self.frames = [json.loads(line) for line in self.motion_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not self.frames:
            raise RuntimeError(f"motion_jsonl is empty: {self.motion_jsonl}")
        self.camera_records_by_group = {}
        self.camera_image_root = None
        if self.publish_camera_images:
            self.camera_records_by_group, self.camera_image_root = self._load_camera_records()

        self.head_pose_pub = self.create_publisher(PoseStamped, "/motion/head_pose", 10)
        self.wrist_pose_pub = self.create_publisher(PoseStamped, "/motion/wrist_pose", 10)
        self.head_path_pub = self.create_publisher(PathMsg, "/motion/head_path", 10)
        self.wrist_path_pub = self.create_publisher(PathMsg, "/motion/wrist_path", 10)
        self.hand_marker_pub = self.create_publisher(MarkerArray, "/motion/hand_skeleton", 10)
        self.camera_image_pubs = {
            camera_id: self.create_publisher(Image, f"/camera/{camera_id}/image_raw", 3)
            for camera_id in ("C0", "C1", "C2", "C3")
        }
        self.tf_broadcaster = TransformBroadcaster(self)

        self.head_path = PathMsg()
        self.head_path.header.frame_id = self.fixed_frame
        self.wrist_path = PathMsg()
        self.wrist_path.header.frame_id = self.fixed_frame
        self.start_monotonic = time.monotonic()
        self.last_published_abs_index = -1
        self.loop_cycle = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(f"Replaying {len(self.frames)} motion frames from {self.motion_jsonl}")
        if self.publish_camera_images and self.camera_records_by_group:
            self.get_logger().info(
                f"Publishing replay camera images from {self.camera_image_root} on /camera/C*/image_raw"
            )

    def _tick(self) -> None:
        abs_index = int((time.monotonic() - self.start_monotonic) * self.rate_hz)
        if abs_index == self.last_published_abs_index:
            return
        if abs_index >= len(self.frames) and not self.loop:
            return
        if self.loop:
            cycle = abs_index // len(self.frames)
            frame_index = abs_index % len(self.frames)
            if cycle != self.loop_cycle:
                self.loop_cycle = cycle
                self.head_path.poses.clear()
                self.wrist_path.poses.clear()
        else:
            frame_index = abs_index

        self.last_published_abs_index = abs_index
        frame = self.frames[frame_index]
        stamp = self.get_clock().now().to_msg()
        self._maybe_publish_camera_images(frame, stamp)
        head_state = frame.get("head")
        if head_state is not None:
            head_pose = self._pose_msg(head_state, stamp)
            self.head_pose_pub.publish(head_pose)
            self._append_path(self.head_path, head_pose)
            self.head_path_pub.publish(self.head_path)
            self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.head_frame, head_state, stamp))

        wrist_state = frame.get("wrist")
        if wrist_state is not None:
            wrist_pose = self._pose_msg(wrist_state, stamp)
            self.wrist_pose_pub.publish(wrist_pose)
            self._append_path(self.wrist_path, wrist_pose)
            self.wrist_path_pub.publish(self.wrist_path)
            self.tf_broadcaster.sendTransform(self._transform_msg(self.fixed_frame, self.wrist_frame, wrist_state, stamp))
        self.hand_marker_pub.publish(self._hand_markers(frame.get("hands") or [], stamp))

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

    def _load_camera_records(self) -> tuple[dict[int, list[dict]], Path | None]:
        frames_jsonl = self.camera_frames_jsonl
        if frames_jsonl is None:
            frames_jsonl = self._infer_camera_frames_jsonl()
        if not frames_jsonl or not frames_jsonl.exists():
            self.get_logger().warn("Camera image replay disabled: cameras/frames.jsonl not found.")
            return {}, None
        if frames_jsonl.is_dir():
            self.get_logger().warn(f"Camera image replay disabled: expected frames.jsonl, got directory {frames_jsonl}")
            return {}, None
        records_by_group: dict[int, list[dict]] = {}
        with frames_jsonl.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                record = json.loads(line)
                records_by_group.setdefault(int(record["group_id"]), []).append(record)
        return records_by_group, frames_jsonl.parent

    def _infer_camera_frames_jsonl(self) -> Path | None:
        summary_path = self.motion_jsonl.parent / "world_anchor_summary.json"
        if summary_path.exists():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                session_dir = Path(str(summary.get("session_dir") or ""))
                if session_dir.name == "cameras":
                    frames_jsonl = session_dir / "frames.jsonl"
                else:
                    frames_jsonl = session_dir / "cameras" / "frames.jsonl"
                if frames_jsonl.exists():
                    return frames_jsonl
            except (OSError, json.JSONDecodeError):
                pass
        parts = self.motion_jsonl.resolve().parts
        if "data" in parts and "processed" in parts:
            try:
                index = parts.index("processed")
                session_name = parts[index + 1]
            except (ValueError, IndexError):
                return None
            return Path(*parts[:index], "raw", session_name, "cameras", "frames.jsonl")
        return None

    def _maybe_publish_camera_images(self, frame: dict, stamp) -> None:
        if not self.publish_camera_images:
            return
        now = time.monotonic()
        if self.camera_publish_period_s > 0.0 and now - self.last_camera_publish_monotonic < self.camera_publish_period_s:
            return
        self.last_camera_publish_monotonic = now
        self._publish_camera_images(frame, stamp)

    def _publish_camera_images(self, frame: dict, stamp) -> None:
        if not self.publish_camera_images or not self.camera_records_by_group or self.camera_image_root is None:
            return
        records = self.camera_records_by_group.get(int(frame.get("group_id", -1)))
        if not records:
            return
        for record in records:
            camera_id = str(record.get("camera_id") or "")
            pub = self.camera_image_pubs.get(camera_id)
            if pub is None:
                continue
            image_path = self.camera_image_root / str(record.get("image_path") or "")
            msg = self._image_msg(image_path, camera_id, stamp)
            if msg is not None:
                try:
                    pub.publish(msg)
                except Exception:
                    return

    def _image_msg(self, image_path: Path, camera_id: str, stamp) -> Image | None:
        try:
            import cv2
        except ImportError:
            self.get_logger().warn("Camera image replay disabled: cv2 is not available.")
            self.publish_camera_images = False
            return None
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            return None
        if self.camera_image_max_width > 0 and image.shape[1] > self.camera_image_max_width:
            scale = float(self.camera_image_max_width) / float(image.shape[1])
            image = cv2.resize(image, (self.camera_image_max_width, int(round(image.shape[0] * scale))))
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = camera_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg

    def _hand_markers(self, hands: list[dict], stamp) -> MarkerArray:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        marker_id = 1
        for hand in hands:
            points_by_index = {
                int(item["index"]): item["world"]
                for item in hand.get("landmarks", [])
                if item.get("world") is not None
            }
            if not points_by_index:
                continue
            lines = Marker()
            lines.header.frame_id = self.fixed_frame
            lines.header.stamp = stamp
            lines.ns = "hand_skeleton"
            lines.id = marker_id
            marker_id += 1
            lines.type = Marker.LINE_LIST
            lines.action = Marker.ADD
            lines.scale.x = 0.006
            lines.color.r = 0.1
            lines.color.g = 0.8
            lines.color.b = 1.0
            lines.color.a = 1.0
            for start, end in hand.get("connections", []):
                if start in points_by_index and end in points_by_index:
                    lines.points.append(self._point(points_by_index[start]))
                    lines.points.append(self._point(points_by_index[end]))
            array.markers.append(lines)

            joints = Marker()
            joints.header.frame_id = self.fixed_frame
            joints.header.stamp = stamp
            joints.ns = "hand_joints"
            joints.id = marker_id
            marker_id += 1
            joints.type = Marker.SPHERE_LIST
            joints.action = Marker.ADD
            joints.scale.x = 0.018
            joints.scale.y = 0.018
            joints.scale.z = 0.018
            joints.color.r = 1.0
            joints.color.g = 0.85
            joints.color.b = 0.15
            joints.color.a = 1.0
            for index in sorted(points_by_index):
                joints.points.append(self._point(points_by_index[index]))
            array.markers.append(joints)
        return array

    @staticmethod
    def _point(values: list[float]) -> Point:
        point = Point()
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2])
        return point


def main() -> None:
    rclpy.init()
    node = MotionReplayVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
