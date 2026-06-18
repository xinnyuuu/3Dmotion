from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


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
    accel_mps2: list[float]
    gyro_radps: list[float]


def prepare_openvins_session(
    session_dir: Path,
    output_dir: Path,
    camera_ids: list[str] | None = None,
    imu_slot: str = "head_imu",
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
    selected_camera_ids = set(camera_ids or ["C0"])
    image_records = _load_image_records(frames_path, cameras_dir, selected_camera_ids)
    imu_records = _load_imu_records(imu_path)

    if not image_records:
        raise RuntimeError(f"No frames found for cameras: {sorted(selected_camera_ids)}")
    if not imu_records:
        raise RuntimeError(f"No IMU samples found in {imu_path}")

    camera_topics = {camera_id: f"/cam{index}/image_raw" for index, camera_id in enumerate(sorted(selected_camera_ids))}
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
        "next_step": "Convert these streams to rosbag2 topics /cam0/image_raw and /imu0, then run OpenVINS subscribe.launch.py.",
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
    for timestamp_ns, kind, record in events:
        if kind == "image":
            image = cv2.imread(str(record["image_path"]), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"Could not read image for rosbag2 export: {record['image_path']}")
            message = _image_msg(Image, Header, image, timestamp_ns, frame_id=str(record["camera_id"]))
            writer.write(str(record["topic"]), serialize_message(message), timestamp_ns)
            written_images += 1
            continue

        message = _imu_msg(Imu, Header, record, timestamp_ns, frame_id=frame_id)
        writer.write(str(record["topic"]), serialize_message(message), timestamp_ns)
        written_imu += 1

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
    }
    summary_path = bag_dir.with_name(f"{bag_dir.name}_summary.json")
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


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


def _load_image_records(frames_path: Path, cameras_dir: Path, camera_ids: set[str]) -> list[dict]:
    records = []
    with frames_path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("camera_id") not in camera_ids:
                continue
            image_path = cameras_dir / record["image_path"]
            if not image_path.exists():
                raise FileNotFoundError(f"Frame image listed in frames.jsonl does not exist: {image_path}")
            records.append(record)
    records.sort(key=lambda item: int(item["timestamp_monotonic_ns"]))
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


def _read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


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
    parser.add_argument("--camera-id", action="append", dest="camera_ids", help="Camera ID to export. Repeat for multiple cameras. Default: C0.")
    parser.add_argument("--imu-slot", default="head_imu", help="IMU slot JSONL under session/imus. Default: head_imu.")
    args = parser.parse_args()

    summary = prepare_openvins_session(
        session_dir=Path(args.session_dir),
        output_dir=Path(args.output_dir),
        camera_ids=args.camera_ids,
        imu_slot=args.imu_slot,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
