from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


def prepare_euroc_openvins_session(
    mav0_dir: Path,
    output_dir: Path,
    *,
    config_source_dir: Path = Path("open_vins/config/euroc_mav"),
) -> dict:
    """Convert a EuRoC MAV `mav0` folder into the local OpenVINS prepared layout."""

    mav0_dir = mav0_dir.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cam0_records = _read_euroc_camera_csv(mav0_dir, "cam0", "/cam0/image_raw")
    cam1_records = _read_euroc_camera_csv(mav0_dir, "cam1", "/cam1/image_raw")
    imu_records = _read_euroc_imu_csv(mav0_dir / "imu0" / "data.csv")
    if not cam0_records:
        raise RuntimeError(f"No cam0 records under {mav0_dir}")
    if not cam1_records:
        raise RuntimeError(f"No cam1 records under {mav0_dir}")
    if not imu_records:
        raise RuntimeError(f"No imu0 records under {mav0_dir}")

    image_records = sorted(cam0_records + cam1_records, key=lambda item: (item["timestamp_unix_ns"], item["camera_id"]))

    images_path = output_dir / "images.jsonl"
    imu_path = output_dir / "imu.jsonl"
    _write_jsonl(images_path, image_records)
    _write_jsonl(imu_path, imu_records)

    config_dir = output_dir / "config"
    _copy_config(config_source_dir, config_dir)

    summary = {
        "dataset": "euroc",
        "mav0_dir": str(mav0_dir),
        "output_dir": str(output_dir),
        "config_dir": str(config_dir),
        "outputs": {
            "images_jsonl": str(images_path),
            "imu_jsonl": str(imu_path),
        },
        "topics": {
            "imu": "/imu0",
            "cameras": {
                "cam0": "/cam0/image_raw",
                "cam1": "/cam1/image_raw",
            },
        },
        "counts": {
            "cam0_images": len(cam0_records),
            "cam1_images": len(cam1_records),
            "imu_samples": len(imu_records),
        },
        "time_range_ns": {
            "images": [int(image_records[0]["timestamp_unix_ns"]), int(image_records[-1]["timestamp_unix_ns"])],
            "imu": [int(imu_records[0]["timestamp_unix_ns"]), int(imu_records[-1]["timestamp_unix_ns"])],
        },
        "next_commands": _next_commands(output_dir),
    }
    (output_dir / "euroc_openvins_session_manifest.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def _read_euroc_camera_csv(mav0_dir: Path, camera_id: str, topic: str) -> list[dict]:
    csv_path = mav0_dir / camera_id / "data.csv"
    image_dir = mav0_dir / camera_id / "data"
    records = []
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.reader(line for line in f if line.strip() and not line.startswith("#")):
            if len(row) < 2:
                continue
            timestamp_ns = int(row[0])
            image_path = image_dir / row[1]
            if not image_path.exists():
                raise FileNotFoundError(f"Missing EuRoC image: {image_path}")
            records.append(
                {
                    "topic": topic,
                    "camera_id": camera_id,
                    "timestamp_unix_ns": timestamp_ns,
                    "timestamp_monotonic_ns": timestamp_ns,
                    "image_path": str(image_path.resolve()),
                    "width": 0,
                    "height": 0,
                }
            )
    return sorted(records, key=lambda item: int(item["timestamp_unix_ns"]))


def _read_euroc_imu_csv(path: Path) -> list[dict]:
    records = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.reader(line for line in f if line.strip() and not line.startswith("#")):
            if len(row) < 7:
                continue
            timestamp_ns = int(row[0])
            records.append(
                {
                    "topic": "/imu0",
                    "sensor_id": "imu0",
                    "timestamp_unix_ns": timestamp_ns,
                    "timestamp_monotonic_ns": timestamp_ns,
                    "gyro_radps": [float(row[1]), float(row[2]), float(row[3])],
                    "accel_mps2": [float(row[4]), float(row[5]), float(row[6])],
                }
            )
    return sorted(records, key=lambda item: int(item["timestamp_unix_ns"]))


def _copy_config(source_dir: Path, output_dir: Path) -> None:
    source_dir = source_dir.resolve()
    if not source_dir.is_dir():
        raise FileNotFoundError(f"Missing OpenVINS EuRoC config directory: {source_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    for name in ["estimator_config.yaml", "kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"]:
        shutil.copyfile(source_dir / name, output_dir / name)


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record, separators=(",", ":")) + "\n")


def _next_commands(output_dir: Path) -> dict:
    config_path = output_dir / "config" / "estimator_config.yaml"
    bag_dir = output_dir / "rosbag2"
    return {
        "run_openvins_stereo": (
            "source scripts/source_openvins_ros2.bash\n"
            "ros2 launch ov_msckf subscribe.launch.py "
            f"config_path:={config_path} "
            "max_cameras:=2 use_stereo:=true"
        ),
        "play_rosbag2": (
            "source /opt/ros/humble/setup.bash\n"
            f"ros2 bag play {bag_dir}"
        ),
        "launch_rviz": (
            "cd ros2_ws\n"
            "source /opt/ros/humble/setup.bash\n"
            "source install/setup.bash\n"
            "ros2 launch vimas_motion_bringup head_vio_rviz.launch.py child_frame:=euroc_imu"
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare a EuRoC MAV dataset for the 3DMotion OpenVINS ROS2 pipeline.")
    parser.add_argument("--mav0-dir", required=True, help="EuRoC mav0 directory, e.g. ../VIO_data/VIO/V2_01_easy/mav0.")
    parser.add_argument("--output-dir", default="data/processed/euroc_v2_01_easy/openvins_stereo")
    parser.add_argument("--config-source-dir", default="open_vins/config/euroc_mav")
    args = parser.parse_args()

    summary = prepare_euroc_openvins_session(
        mav0_dir=Path(args.mav0_dir),
        output_dir=Path(args.output_dir),
        config_source_dir=Path(args.config_source_dir),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
