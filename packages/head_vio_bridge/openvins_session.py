from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from packages.head_vio_bridge.openvins_config import DEFAULT_HEAD_CAMERA_IDS


@dataclass
class OpenVinsImageRecord:
    topic: str
    camera_id: str
    timestamp_unix_ns: int
    timestamp_monotonic_ns: int
    image_path: str
    width: int
    height: int


@dataclass
class OpenVinsImuRecord:
    topic: str
    sensor_id: str
    timestamp_unix_ns: int
    timestamp_monotonic_ns: int
    timestamp_source: str
    accel_mps2: list[float]
    gyro_radps: list[float]


def prepare_openvins_session(
    session_dir: Path,
    output_dir: Path,
    camera_ids: list[str] | None = None,
    imu_slot: str = "head_imu",
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    export_window: bool = False,
    export_start_offset_s: float = 0.0,
    export_max_duration_s: float | None = None,
    export_imu_preroll_s: float = 0.0,
) -> dict:
    """Validate a recorded session and export OpenVINS-friendly JSONL streams.

    This is intentionally ROS-free. It creates a clean intermediate layout that
    can later be converted to rosbag2 without tying the capture dashboard to a
    ROS environment.
    """

    session_dir = session_dir.resolve()
    cameras_dir = _resolve_cameras_dir(session_dir)
    imus_dir = session_dir / "imus"
    frames_path = cameras_dir / "frames.jsonl"
    imu_path = imus_dir / f"{imu_slot}.jsonl"

    if not frames_path.exists():
        raise FileNotFoundError(f"Missing camera frame manifest: {frames_path}")
    if not imu_path.exists():
        raise FileNotFoundError(f"Missing head IMU log: {imu_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    selected_camera_ids = list(camera_ids or DEFAULT_HEAD_CAMERA_IDS)
    image_records = _load_image_records(frames_path, cameras_dir, selected_camera_ids)
    imu_records = _load_imu_records(imu_path)
    imu_time_summary = _summarize_imu_timing(imu_records)
    export_filter_summary = {
        "enabled": False,
        "start_offset_s": export_start_offset_s,
        "max_duration_s": export_max_duration_s,
        "imu_preroll_s": export_imu_preroll_s,
    }
    if export_window:
        image_records, imu_records, export_filter_summary = _filter_export_window(
            image_records=image_records,
            imu_records=imu_records,
            start_offset_s=export_start_offset_s,
            max_duration_s=export_max_duration_s,
            imu_preroll_s=export_imu_preroll_s,
        )
    if imu_time_mode == "resample-rate":
        imu_records = _resample_imu_records(imu_records, rate_hz=imu_rate_hz)
    elif imu_time_mode == "reconstruct-rate":
        imu_records = _reconstruct_imu_records_by_rate(imu_records, rate_hz=imu_rate_hz)
    elif imu_time_mode != "raw":
        raise ValueError("--imu-time-mode must be one of: raw, resample-rate, reconstruct-rate")
    exported_imu_time_summary = _summarize_imu_timing(imu_records)

    if not image_records:
        raise RuntimeError(f"No frames found for cameras: {sorted(selected_camera_ids)}")
    if not imu_records:
        raise RuntimeError(f"No IMU samples found in {imu_path}")

    camera_topics = {camera_id: f"/cam{index}/image_raw" for index, camera_id in enumerate(selected_camera_ids)}
    imu_topic = "/imu0"

    image_output = output_dir / "images.jsonl"
    imu_output = output_dir / "imu.jsonl"
    with image_output.open("w", encoding="utf-8") as f:
        for record in image_records:
            exported = OpenVinsImageRecord(
                topic=camera_topics[record["camera_id"]],
                camera_id=record["camera_id"],
                timestamp_unix_ns=int(record["timestamp_unix_ns"]),
                timestamp_monotonic_ns=int(record["timestamp_monotonic_ns"]),
                image_path=str((cameras_dir / record["image_path"]).resolve()),
                width=int(record["width"]),
                height=int(record["height"]),
            )
            f.write(json.dumps(asdict(exported), separators=(",", ":")) + "\n")

    with imu_output.open("w", encoding="utf-8") as f:
        for record in imu_records:
            exported = OpenVinsImuRecord(
                topic=imu_topic,
                sensor_id=str(record.get("sensor_id") or imu_slot),
                timestamp_unix_ns=int(record["timestamp_unix_ns"]),
                timestamp_monotonic_ns=int(record["timestamp_monotonic_ns"]),
                timestamp_source=str(record.get("timestamp_source") or "unknown"),
                accel_mps2=[float(v) for v in record["accel_mps2"]],
                gyro_radps=[float(v) for v in record["gyro_radps"]],
            )
            f.write(json.dumps(asdict(exported), separators=(",", ":")) + "\n")

    summary = {
        "session_dir": str(session_dir),
        "source": {
            "frames_jsonl": str(frames_path),
            "imu_jsonl": str(imu_path),
        },
        "openvins_topics": {
            "imu": imu_topic,
            "cameras": camera_topics,
        },
        "outputs": {
            "images_jsonl": str(image_output),
            "imu_jsonl": str(imu_output),
        },
        "counts": {
            "images": len(image_records),
            "imu_samples": len(imu_records),
        },
        "imu_time": {
            "mode": imu_time_mode,
            "rate_hz": imu_rate_hz,
            "source": imu_time_summary,
            "exported": exported_imu_time_summary,
        },
        "export_window": export_filter_summary,
        "time_range_monotonic_ns": {
            "images": [int(image_records[0]["timestamp_monotonic_ns"]), int(image_records[-1]["timestamp_monotonic_ns"])],
            "imu": [int(imu_records[0]["timestamp_monotonic_ns"]), int(imu_records[-1]["timestamp_monotonic_ns"])],
        },
        "p3a_frame_convention": {
            "openvins_body_frame": imu_slot,
            "prototype_head_frame": "H := I",
            "prototype_output": "T_W_H := T_W_I",
            "note": "This is valid only when head_imu is rigidly fixed to the headset/camera rig.",
        },
        "next_step": "Run OpenVINS with the ROS-free CSV/image runner, or convert these streams to rosbag2 for ROS2/RViz debugging.",
    }
    summary_path = output_dir / "openvins_session_manifest.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def write_openvins_rosbag2(
    prepared_dir: Path,
    bag_dir: Path,
    frame_id: str = "headset",
    *,
    max_duration_s: float | None = None,
    start_offset_s: float = 0.0,
    image_stride: int = 1,
    progress_every: int = 500,
) -> dict:
    """Write prepared OpenVINS JSONL streams to a rosbag2 directory.

    This function requires a sourced ROS2 Python environment. It is imported
    lazily so the rest of the project can still run inside a plain venv.
    """

    try:
        import cv2
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Image, Imu
        from std_msgs.msg import Header
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python packages are not available. Source your ROS2/OpenVINS workspace "
            "before running rosbag2 export, e.g. `source /opt/ros/humble/setup.bash`."
        ) from exc

    images_path = prepared_dir / "images.jsonl"
    imu_path = prepared_dir / "imu.jsonl"
    if not images_path.exists():
        raise FileNotFoundError(f"Missing prepared image stream: {images_path}")
    if not imu_path.exists():
        raise FileNotFoundError(f"Missing prepared IMU stream: {imu_path}")

    image_records = list(_read_jsonl(images_path))
    imu_records = list(_read_jsonl(imu_path))
    if not image_records:
        raise RuntimeError(f"No image records in {images_path}")
    if not imu_records:
        raise RuntimeError(f"No IMU records in {imu_path}")
    image_records, imu_records, filter_summary = _filter_rosbag_records(
        image_records,
        imu_records,
        max_duration_s=max_duration_s,
        start_offset_s=start_offset_s,
        image_stride=image_stride,
    )
    if not image_records:
        raise RuntimeError("No image records remain after rosbag2 export filters")
    if not imu_records:
        raise RuntimeError("No IMU records remain after rosbag2 export filters")

    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    image_topics = sorted({str(record["topic"]) for record in image_records})
    for topic in image_topics:
        writer.create_topic(_topic_metadata(rosbag2_py, topic, "sensor_msgs/msg/Image"))
    imu_topic = str(imu_records[0]["topic"])
    writer.create_topic(_topic_metadata(rosbag2_py, imu_topic, "sensor_msgs/msg/Imu"))

    events = []
    for record in image_records:
        events.append((int(record["timestamp_unix_ns"]), "image", record))
    for record in imu_records:
        events.append((int(record["timestamp_unix_ns"]), "imu", record))
    events.sort(key=lambda item: item[0])

    written_images = 0
    written_imu = 0
    total_events = len(events)
    for event_index, (timestamp_ns, kind, record) in enumerate(events, start=1):
        if kind == "image":
            image = cv2.imread(str(record["image_path"]), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"Could not read image for rosbag2 export: {record['image_path']}")
            message = _image_msg(Image, Header, image, timestamp_ns, frame_id=str(record["camera_id"]))
            writer.write(str(record["topic"]), serialize_message(message), timestamp_ns)
            written_images += 1
        else:
            message = _imu_msg(Imu, Header, record, timestamp_ns, frame_id=frame_id)
            writer.write(str(record["topic"]), serialize_message(message), timestamp_ns)
            written_imu += 1
        if progress_every > 0 and (event_index % progress_every == 0 or event_index == total_events):
            percent = 100.0 * event_index / max(total_events, 1)
            print(
                f"[rosbag2 export] {event_index}/{total_events} events ({percent:.1f}%), "
                f"images={written_images}, imu={written_imu}",
                file=sys.stderr,
                flush=True,
            )

    summary = {
        "bag_dir": str(bag_dir),
        "topics": {
            "images": image_topics,
            "imu": imu_topic,
        },
        "counts": {
            "images": written_images,
            "imu_samples": written_imu,
        },
        "filters": filter_summary,
        "progress_every": progress_every,
    }
    summary_path = bag_dir.with_name(f"{bag_dir.name}_summary.json")
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def replay_openvins_session_ros2(
    prepared_dir: Path,
    *,
    frame_id: str = "headset",
    max_duration_s: float | None = None,
    start_offset_s: float = 0.0,
    image_stride: int = 1,
    rate: float = 0.0,
    start_delay_s: float = 2.0,
    progress_every: int = 500,
    head_pose_topic: str = "/ov_msckf/poseimu",
    head_pose_path: Path | None = None,
    reliability: str = "reliable",
) -> dict:
    """Replay prepared OpenVINS JSONL streams directly to ROS2 topics.

    This avoids writing a large intermediate rosbag2. It still requires a
    sourced ROS2/OpenVINS Python environment because it publishes
    sensor_msgs/Image and sensor_msgs/Imu messages.
    """

    try:
        import cv2
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, Imu
        from std_msgs.msg import Header
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python packages are not available. Source your ROS2/OpenVINS workspace "
            "before replay and use the system ROS Python, e.g. "
            "`source scripts/source_openvins_ros2.bash` then "
            "`/usr/bin/python3 scripts/session_replay_publisher.py --prepared-dir <openvins_head>`."
        ) from exc

    if rate < 0:
        raise ValueError("--rate must be >= 0. Use 0 for fastest possible replay.")
    normalized_reliability = reliability.strip().lower().replace("-", "_")
    if normalized_reliability not in {"reliable", "best_effort"}:
        raise ValueError("--reliability must be 'reliable' or 'best_effort'")

    images_path = prepared_dir / "images.jsonl"
    imu_path = prepared_dir / "imu.jsonl"
    if not images_path.exists():
        raise FileNotFoundError(f"Missing prepared image stream: {images_path}")
    if not imu_path.exists():
        raise FileNotFoundError(f"Missing prepared IMU stream: {imu_path}")

    image_records = list(_read_jsonl(images_path))
    imu_records = list(_read_jsonl(imu_path))
    if not image_records:
        raise RuntimeError(f"No image records in {images_path}")
    if not imu_records:
        raise RuntimeError(f"No IMU records in {imu_path}")
    image_records, imu_records, filter_summary = _filter_rosbag_records(
        image_records,
        imu_records,
        max_duration_s=max_duration_s,
        start_offset_s=start_offset_s,
        image_stride=image_stride,
    )
    if not image_records:
        raise RuntimeError("No image records remain after replay filters")
    if not imu_records:
        raise RuntimeError("No IMU records remain after replay filters")

    image_topics = sorted({str(record["topic"]) for record in image_records})
    imu_topic = str(imu_records[0]["topic"])
    events = [(int(record["timestamp_unix_ns"]), "image", record) for record in image_records]
    events.extend((int(record["timestamp_unix_ns"]), "imu", record) for record in imu_records)
    events.sort(key=lambda item: item[0])

    pose_writer = _HeadPoseJsonlWriter(
        head_pose_path or (prepared_dir / "head_pose.jsonl"),
        imu_records=imu_records,
    )

    rclpy.init(args=None)
    node = rclpy.create_node("openvins_session_replay")
    try:
        reliability_policy = (
            ReliabilityPolicy.RELIABLE
            if normalized_reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        replay_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability_policy,
        )
        print(
            f"[session replay] QoS reliability={normalized_reliability}, depth=10",
            file=sys.stderr,
            flush=True,
        )
        image_publishers = {
            topic: node.create_publisher(Image, topic, replay_qos)
            for topic in image_topics
        }
        imu_publisher = node.create_publisher(Imu, imu_topic, replay_qos)
        pose_subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            head_pose_topic,
            pose_writer.callback,
            100,
        )
        _ = pose_subscription

        if start_delay_s > 0:
            deadline = time.monotonic() + start_delay_s
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        first_event_ns = events[0][0]
        wall_start = time.monotonic()
        published_images = 0
        published_imu = 0
        total_events = len(events)

        for event_index, (timestamp_ns, kind, record) in enumerate(events, start=1):
            if not rclpy.ok():
                break
            if rate > 0:
                target_delay_s = (timestamp_ns - first_event_ns) / 1_000_000_000 / rate
                while rclpy.ok():
                    remaining_s = wall_start + target_delay_s - time.monotonic()
                    if remaining_s <= 0:
                        break
                    rclpy.spin_once(node, timeout_sec=min(0.02, remaining_s))

            if kind == "image":
                image = cv2.imread(str(record["image_path"]), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f"Could not read image for replay: {record['image_path']}")
                message = _image_msg(Image, Header, image, timestamp_ns, frame_id=str(record["camera_id"]))
                image_publishers[str(record["topic"])].publish(message)
                published_images += 1
            else:
                message = _imu_msg(Imu, Header, record, timestamp_ns, frame_id=frame_id)
                imu_publisher.publish(message)
                published_imu += 1

            rclpy.spin_once(node, timeout_sec=0.0)
            if progress_every > 0 and (event_index % progress_every == 0 or event_index == total_events):
                percent = 100.0 * event_index / max(total_events, 1)
                print(
                    f"[session replay] {event_index}/{total_events} events ({percent:.1f}%), "
                    f"images={published_images}, imu={published_imu}, poses={pose_writer.count}",
                    file=sys.stderr,
                    flush=True,
                )

        drain_until = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < drain_until:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        pose_writer.close()
        node.destroy_node()
        _shutdown_rclpy_once(rclpy)

    summary = {
        "prepared_dir": str(prepared_dir),
        "topics": {
            "images": image_topics,
            "imu": imu_topic,
            "head_pose": head_pose_topic,
        },
        "counts": {
            "images": published_images,
            "imu_samples": published_imu,
            "head_poses": pose_writer.count,
        },
        "filters": filter_summary,
        "rate": rate,
        "start_delay_s": start_delay_s,
        "reliability": normalized_reliability,
        "head_pose_jsonl": str(pose_writer.path),
    }
    summary_path = prepared_dir / "session_replay_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _shutdown_rclpy_once(rclpy_module) -> None:
    try:
        if rclpy_module.ok():
            rclpy_module.shutdown()
    except Exception as exc:
        if "rcl_shutdown already called" not in str(exc):
            raise


class _HeadPoseJsonlWriter:
    def __init__(self, path: Path, *, imu_records: list[dict]) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8")
        self._offset_ns = _unix_minus_monotonic_offset_ns(imu_records)
        self.count = 0

    def callback(self, message) -> None:
        timestamp_unix_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        timestamp_monotonic_ns = timestamp_unix_ns - self._offset_ns if self._offset_ns is not None else 0
        pose = message.pose.pose
        record = {
            "timestamp_unix_ns": timestamp_unix_ns,
            "timestamp_monotonic_ns": timestamp_monotonic_ns,
            "timestamp_source": "openvins_ros2_poseimu",
            "tracking_state": 1,
            "T_W_H": {
                "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "orientation_wxyz": [
                    float(pose.orientation.w),
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                ],
            },
        }
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.count += 1

    def close(self) -> None:
        self._file.close()


def _filter_rosbag_records(
    image_records: list[dict],
    imu_records: list[dict],
    *,
    max_duration_s: float | None,
    start_offset_s: float,
    image_stride: int,
) -> tuple[list[dict], list[dict], dict]:
    if image_stride < 1:
        raise ValueError("--image-stride must be >= 1")
    if start_offset_s < 0:
        raise ValueError("--start-offset-s must be >= 0")
    if max_duration_s is not None and max_duration_s <= 0:
        raise ValueError("--max-duration-s must be > 0")

    first_timestamp_ns = min(int(record["timestamp_unix_ns"]) for record in image_records)
    start_ns = first_timestamp_ns + int(start_offset_s * 1_000_000_000)
    end_ns = None if max_duration_s is None else start_ns + int(max_duration_s * 1_000_000_000)

    def in_window(record: dict) -> bool:
        timestamp_ns = int(record["timestamp_unix_ns"])
        if timestamp_ns < start_ns:
            return False
        if end_ns is not None and timestamp_ns > end_ns:
            return False
        return True

    windowed_images = [record for record in image_records if in_window(record)]
    windowed_imu = [record for record in imu_records if in_window(record)]
    strided_images = [record for index, record in enumerate(windowed_images) if index % image_stride == 0]

    return (
        strided_images,
        windowed_imu,
        {
            "start_offset_s": start_offset_s,
            "max_duration_s": max_duration_s,
            "image_stride": image_stride,
            "input_counts": {
                "images": len(image_records),
                "imu_samples": len(imu_records),
            },
            "window_counts": {
                "images": len(windowed_images),
                "imu_samples": len(windowed_imu),
            },
            "output_counts": {
                "images": len(strided_images),
                "imu_samples": len(windowed_imu),
            },
        },
    )


def _resolve_cameras_dir(session_dir: Path) -> Path:
    if (session_dir / "cameras").is_dir():
        return session_dir / "cameras"
    return session_dir


def _load_image_records(frames_path: Path, cameras_dir: Path, camera_ids: list[str]) -> list[dict]:
    camera_id_set = set(camera_ids)
    records = []
    with frames_path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("camera_id") not in camera_id_set:
                continue
            image_path = cameras_dir / record["image_path"]
            if not image_path.exists():
                raise FileNotFoundError(f"Frame image listed in frames.jsonl does not exist: {image_path}")
            records.append(record)
    order = {camera_id: index for index, camera_id in enumerate(camera_ids)}
    records.sort(key=lambda item: (int(item["timestamp_monotonic_ns"]), order.get(str(item["camera_id"]), 999)))
    return records


def _load_imu_records(path: Path) -> list[dict]:
    records = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            if "accel_mps2" in record and "gyro_radps" in record:
                records.append(record)
    records.sort(key=lambda item: int(item["timestamp_monotonic_ns"]))
    return records


def _filter_export_window(
    *,
    image_records: list[dict],
    imu_records: list[dict],
    start_offset_s: float,
    max_duration_s: float | None,
    imu_preroll_s: float,
) -> tuple[list[dict], list[dict], dict]:
    if start_offset_s < 0:
        raise ValueError("--start-offset-s must be >= 0")
    if max_duration_s is not None and max_duration_s <= 0:
        raise ValueError("--max-duration-s must be > 0")
    if imu_preroll_s < 0:
        raise ValueError("--imu-preroll-s must be >= 0")
    if not image_records or not imu_records:
        return image_records, imu_records, {
            "enabled": True,
            "start_offset_s": start_offset_s,
            "max_duration_s": max_duration_s,
            "imu_preroll_s": imu_preroll_s,
            "input_counts": {"images": len(image_records), "imu_samples": len(imu_records)},
            "output_counts": {"images": len(image_records), "imu_samples": len(imu_records)},
        }

    first_image_ns = min(int(record["timestamp_monotonic_ns"]) for record in image_records)
    start_ns = first_image_ns + int(round(start_offset_s * 1_000_000_000))
    end_ns = None if max_duration_s is None else start_ns + int(round(max_duration_s * 1_000_000_000))
    imu_start_ns = start_ns - int(round(imu_preroll_s * 1_000_000_000))

    def image_in_window(record: dict) -> bool:
        timestamp_ns = int(record["timestamp_monotonic_ns"])
        return timestamp_ns >= start_ns and (end_ns is None or timestamp_ns <= end_ns)

    def imu_in_window(record: dict) -> bool:
        timestamp_ns = int(record["timestamp_monotonic_ns"])
        return timestamp_ns >= imu_start_ns and (end_ns is None or timestamp_ns <= end_ns)

    filtered_images = [record for record in image_records if image_in_window(record)]
    filtered_imu = [record for record in imu_records if imu_in_window(record)]
    return filtered_images, filtered_imu, {
        "enabled": True,
        "start_offset_s": start_offset_s,
        "max_duration_s": max_duration_s,
        "imu_preroll_s": imu_preroll_s,
        "start_monotonic_ns": start_ns,
        "end_monotonic_ns": end_ns,
        "imu_start_monotonic_ns": imu_start_ns,
        "input_counts": {"images": len(image_records), "imu_samples": len(imu_records)},
        "output_counts": {"images": len(filtered_images), "imu_samples": len(filtered_imu)},
    }


def _summarize_imu_timing(records: list[dict]) -> dict:
    if len(records) < 2:
        return {
            "count": len(records),
            "duration_s": 0.0,
            "rate_hz": 0.0,
            "dt_ms_median": None,
            "dt_ms_p99": None,
            "dt_ms_max": None,
            "gaps_over_20ms": 0,
        }
    timestamps = [int(record["timestamp_monotonic_ns"]) for record in records]
    dts_ms = [(curr - prev) / 1e6 for prev, curr in zip(timestamps, timestamps[1:]) if curr > prev]
    duration_s = (timestamps[-1] - timestamps[0]) / 1e9
    return {
        "count": len(records),
        "duration_s": duration_s,
        "rate_hz": (len(records) - 1) / duration_s if duration_s > 0 else 0.0,
        "dt_ms_median": statistics.median(dts_ms) if dts_ms else None,
        "dt_ms_p99": _percentile(dts_ms, 99) if dts_ms else None,
        "dt_ms_max": max(dts_ms) if dts_ms else None,
        "gaps_over_20ms": sum(dt > 20.0 for dt in dts_ms),
    }


def _resample_imu_records(records: list[dict], *, rate_hz: float) -> list[dict]:
    if len(records) < 2:
        return records
    if rate_hz <= 0:
        raise ValueError("--imu-rate-hz must be > 0")

    sorted_records = sorted(records, key=lambda item: int(item["timestamp_monotonic_ns"]))
    source_times = [int(record["timestamp_monotonic_ns"]) for record in sorted_records]
    period_ns = int(round(1_000_000_000 / rate_hz))
    start_ns = source_times[0]
    end_ns = source_times[-1]
    unix_offset_ns = _unix_minus_monotonic_offset_ns(sorted_records) or 0

    resampled = []
    src_index = 0
    timestamp_ns = start_ns
    while timestamp_ns <= end_ns:
        while src_index + 1 < len(sorted_records) and source_times[src_index + 1] < timestamp_ns:
            src_index += 1
        left = sorted_records[src_index]
        right = sorted_records[min(src_index + 1, len(sorted_records) - 1)]
        left_t = int(left["timestamp_monotonic_ns"])
        right_t = int(right["timestamp_monotonic_ns"])
        alpha = 0.0 if right_t == left_t else (timestamp_ns - left_t) / (right_t - left_t)
        copied = dict(left)
        copied["timestamp_monotonic_ns"] = timestamp_ns
        copied["timestamp_unix_ns"] = timestamp_ns + unix_offset_ns
        copied["timestamp_source"] = f"offline_resampled_{rate_hz:g}hz"
        copied["accel_mps2"] = _lerp_vec(left["accel_mps2"], right["accel_mps2"], alpha)
        copied["gyro_radps"] = _lerp_vec(left["gyro_radps"], right["gyro_radps"], alpha)
        copied["resampled_from"] = {
            "left_timestamp_monotonic_ns": left_t,
            "right_timestamp_monotonic_ns": right_t,
            "alpha": alpha,
        }
        resampled.append(copied)
        timestamp_ns += period_ns
    return resampled


def _reconstruct_imu_records_by_rate(records: list[dict], *, rate_hz: float) -> list[dict]:
    if len(records) < 2:
        return records
    if rate_hz <= 0:
        raise ValueError("--imu-rate-hz must be > 0")

    sorted_records = sorted(records, key=lambda item: int(item["timestamp_monotonic_ns"]))
    period_ns = int(round(1_000_000_000 / rate_hz))
    start_ns = int(sorted_records[0]["timestamp_monotonic_ns"])
    unix_offset_ns = _unix_minus_monotonic_offset_ns(sorted_records) or 0

    reconstructed = []
    for index, record in enumerate(sorted_records):
        timestamp_ns = start_ns + index * period_ns
        copied = dict(record)
        copied["timestamp_monotonic_ns"] = timestamp_ns
        copied["timestamp_unix_ns"] = timestamp_ns + unix_offset_ns
        copied["timestamp_source"] = f"offline_reconstructed_{rate_hz:g}hz_by_sample_index"
        copied["timestamp_reconstruction"] = {
            "mode": "sample_index",
            "sample_index": index,
            "sample_rate_hz": rate_hz,
            "source_timestamp_monotonic_ns": int(record["timestamp_monotonic_ns"]),
        }
        reconstructed.append(copied)
    return reconstructed


def _lerp_vec(left: list[float], right: list[float], alpha: float) -> list[float]:
    return [float(a) + (float(b) - float(a)) * float(alpha) for a, b in zip(left, right)]


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percentile / 100.0)
    return ordered[index]


def _read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def _unix_minus_monotonic_offset_ns(records: list[dict]) -> int | None:
    offsets = []
    for record in records[: min(len(records), 100)]:
        if "timestamp_unix_ns" not in record or "timestamp_monotonic_ns" not in record:
            continue
        offsets.append(int(record["timestamp_unix_ns"]) - int(record["timestamp_monotonic_ns"]))
    if not offsets:
        return None
    return int(round(sum(offsets) / len(offsets)))


def _topic_metadata(rosbag2_py, name: str, message_type: str):
    try:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=message_type,
            serialization_format="cdr",
            offered_qos_profiles="",
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=message_type,
            serialization_format="cdr",
        )


def _stamp_from_ns(timestamp_ns: int):
    seconds, nanoseconds = divmod(int(timestamp_ns), 1_000_000_000)
    return seconds, nanoseconds


def _image_msg(image_cls, header_cls, image, timestamp_ns: int, frame_id: str):
    height, width = image.shape[:2]
    seconds, nanoseconds = _stamp_from_ns(timestamp_ns)
    message = image_cls()
    message.header = header_cls()
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = frame_id
    message.height = int(height)
    message.width = int(width)
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = int(width * 3)
    message.data = image.tobytes()
    return message


def _imu_msg(imu_cls, header_cls, record: dict, timestamp_ns: int, frame_id: str):
    seconds, nanoseconds = _stamp_from_ns(timestamp_ns)
    message = imu_cls()
    message.header = header_cls()
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = frame_id
    message.linear_acceleration.x = float(record["accel_mps2"][0])
    message.linear_acceleration.y = float(record["accel_mps2"][1])
    message.linear_acceleration.z = float(record["accel_mps2"][2])
    message.angular_velocity.x = float(record["gyro_radps"][0])
    message.angular_velocity.y = float(record["gyro_radps"][1])
    message.angular_velocity.z = float(record["gyro_radps"][2])
    message.orientation_covariance[0] = -1.0
    return message


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare a recorded session for the next OpenVINS rosbag2 conversion step.")
    parser.add_argument("--session-dir", required=True, help="Dashboard session directory, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--output-dir", default="data/processed/openvins_session", help="Output directory.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        help="Camera ID to export. Repeat for multiple cameras. Default priority: C1,C2,C0,C3.",
    )
    parser.add_argument("--imu-slot", default="head_imu", help="IMU slot JSONL under session/imus. Default: head_imu.")
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Export from this many seconds after the first selected camera frame.")
    parser.add_argument("--max-duration-s", type=float, help="Export only this many seconds after --start-offset-s.")
    parser.add_argument("--imu-preroll-s", type=float, default=0.0, help="When exporting a window, keep this many seconds of IMU before the first exported image.")
    args = parser.parse_args()

    summary = prepare_openvins_session(
        session_dir=Path(args.session_dir),
        output_dir=Path(args.output_dir),
        camera_ids=args.camera_ids,
        imu_slot=args.imu_slot,
        imu_time_mode=args.imu_time_mode,
        imu_rate_hz=args.imu_rate_hz,
        export_window=args.start_offset_s > 0 or args.max_duration_s is not None,
        export_start_offset_s=args.start_offset_s,
        export_max_duration_s=args.max_duration_s,
        export_imu_preroll_s=args.imu_preroll_s,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
