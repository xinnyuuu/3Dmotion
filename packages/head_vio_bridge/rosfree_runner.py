from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
from pathlib import Path
from typing import Iterable

from packages.head_vio_bridge.openvins_config import DEFAULT_HEAD_CAMERA_IDS
from packages.head_vio_bridge.openvins_session import _filter_rosbag_records
from packages.head_vio_bridge.p3_head_vio import prepare_p3_head_vio


DEFAULT_ROSFREE_RUNNER = Path(__file__).resolve().parents[2] / "open_vins/ov_msckf/build_local/run_csv_msckf"


def process_head_vio_rosfree(
    session_dir: Path,
    *,
    output_dir: Path | None = None,
    cameras_path: Path = Path("configs/cameras.yaml"),
    template_config_dir: Path | None = Path("open_vins/config/euroc_mav"),
    kalibr_imucam_path: Path | None = None,
    camera_ids: list[str] | None = None,
    imu_slot: str = "head_imu",
    runner_path: Path = DEFAULT_ROSFREE_RUNNER,
    fail_on_not_ready: bool = True,
    init_imu_thresh: float = 0.5,
    init_max_disparity: float = 10.0,
    init_dyn_use: bool = False,
    timeshift_cam_imu: float | None = None,
    calib_cam_timeoffset: bool = False,
    calib_cam_extrinsics: bool = False,
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    max_duration_s: float | None = None,
    start_offset_s: float = 0.0,
    imu_preroll_s: float = 2.0,
    image_stride: int = 1,
    run_openvins: bool = True,
) -> dict:
    """Prepare and optionally run the ROS-free OpenVINS CSV/image path."""

    session_dir = session_dir.resolve()
    selected_camera_ids = list(camera_ids or DEFAULT_HEAD_CAMERA_IDS)
    if not 1 <= len(selected_camera_ids) <= 4:
        raise ValueError("ROS-free run_csv_msckf currently supports one to four cameras.")

    output_dir = (output_dir or Path("data/processed") / session_dir.name / "openvins_head").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    export_window = start_offset_s > 0 or max_duration_s is not None

    p3_summary = prepare_p3_head_vio(
        session_dir=session_dir,
        output_dir=output_dir,
        config_dir=output_dir / "config",
        cameras_path=cameras_path,
        template_config_dir=template_config_dir,
        kalibr_imucam_path=kalibr_imucam_path,
        camera_ids=selected_camera_ids,
        imu_slot=imu_slot,
        fail_on_not_ready=fail_on_not_ready,
        init_imu_thresh=init_imu_thresh,
        init_max_disparity=init_max_disparity,
        init_dyn_use=init_dyn_use,
        timeshift_cam_imu=timeshift_cam_imu,
        calib_cam_timeoffset=calib_cam_timeoffset,
        calib_cam_extrinsics=calib_cam_extrinsics,
        imu_time_mode=imu_time_mode,
        imu_rate_hz=imu_rate_hz,
        start_offset_s=start_offset_s,
        max_duration_s=max_duration_s,
        export_window=export_window,
        export_imu_preroll_s=imu_preroll_s if export_window else 0.0,
    )

    rosfree_summary = {
        "ok": False,
        "skipped": True,
        "reason": "P3a readiness failed; ROS-free inputs were not prepared.",
    }
    if bool(p3_summary.get("ready_for_p3a")) or not fail_on_not_ready:
        rosfree_summary = _prepare_rosfree_inputs(
            prepared_dir=output_dir,
            camera_ids=selected_camera_ids,
            max_duration_s=None if export_window else max_duration_s,
            start_offset_s=0.0 if export_window else start_offset_s,
            imu_preroll_s=imu_preroll_s,
            image_stride=image_stride,
        )

    run_summary = {
        "ok": False,
        "skipped": True,
        "reason": "Disabled by --prepare-only.",
    }
    if run_openvins and (bool(p3_summary.get("ready_for_p3a")) or not fail_on_not_ready):
        run_summary = _run_openvins_rosfree(
            prepared_dir=output_dir,
            runner_path=runner_path,
            stereo=len(selected_camera_ids) >= 2,
        )
    elif run_openvins:
        run_summary = {
            "ok": False,
            "skipped": True,
            "reason": "P3a readiness failed; ROS-free OpenVINS run was skipped.",
        }

    summary = {
        "session_dir": str(session_dir),
        "output_dir": str(output_dir),
        "ready_for_p3a": bool(p3_summary.get("ready_for_p3a")),
        "ok": bool(p3_summary.get("ready_for_p3a")) and (not run_openvins or run_summary.get("ok", False)),
        "camera_ids": selected_camera_ids,
        "imu_slot": imu_slot,
        "init_imu_thresh": init_imu_thresh,
        "init_max_disparity": init_max_disparity,
        "init_dyn_use": init_dyn_use,
        "timeshift_cam_imu": timeshift_cam_imu,
        "calib_cam_timeoffset": calib_cam_timeoffset,
        "calib_cam_extrinsics": calib_cam_extrinsics,
        "imu_time_mode": imu_time_mode,
        "imu_rate_hz": imu_rate_hz,
        "export_windowed_before_time_sync": export_window,
        "steps": {
            "p3_head_vio": p3_summary,
            "rosfree_inputs": rosfree_summary,
            "rosfree_run": run_summary,
        },
        "outputs": {
            "head_pose": str(output_dir / "head_pose.jsonl"),
            "trajectory_csv": str(output_dir / "rosfree" / "openvins_trajectory.csv"),
        },
        "next_commands": {
            "fuse_motion": (
                "python -m packages.session_tools.motion_fusion "
                f"--session-dir {session_dir} "
                f"--output-root {output_dir.parent} "
                f"--head-pose {output_dir / 'head_pose.jsonl'}"
            ),
            "motion_replay_rviz": (
                "cd ros2_ws\n"
                "source /opt/ros/humble/setup.bash\n"
                "source install/setup.bash\n"
                "ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py "
                f"motion_jsonl:={output_dir.parent / 'motion' / 'motion_frame.jsonl'}"
            ),
        },
        "notes": [
            "ROS-free OpenVINS reads JPG frames in memory and does not write a raw image rosbag2.",
            "The current run_csv_msckf runner supports one to four cameras; default is C1,C2,C0,C3.",
            "head_pose.jsonl uses the same T_W_H schema as the ROS2 RViz bridge, so motion fusion is unchanged.",
        ],
    }
    (output_dir / "head_vio_rosfree_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _prepare_rosfree_inputs(
    *,
    prepared_dir: Path,
    camera_ids: list[str],
    max_duration_s: float | None,
    start_offset_s: float,
    imu_preroll_s: float,
    image_stride: int,
) -> dict:
    if image_stride <= 0:
        raise ValueError("--image-stride must be >= 1")
    if imu_preroll_s < 0:
        raise ValueError("--imu-preroll-s must be >= 0")
    image_records = list(_read_jsonl(prepared_dir / "images.jsonl"))
    all_imu_records = list(_read_jsonl(prepared_dir / "imu.jsonl"))
    image_records, filtered_imu_records, filter_summary = _filter_rosbag_records(
        image_records,
        all_imu_records,
        max_duration_s=max_duration_s,
        start_offset_s=start_offset_s,
        image_stride=1,
    )
    imu_records = _restore_imu_preroll(
        all_imu_records=all_imu_records,
        filtered_imu_records=filtered_imu_records,
        image_records=image_records,
        imu_preroll_s=imu_preroll_s,
    )
    if not image_records:
        raise RuntimeError("No image records remain after ROS-free export filters")
    if not imu_records:
        raise RuntimeError("No IMU records remain after ROS-free export filters")

    rosfree_dir = prepared_dir / "rosfree"
    inputs_dir = rosfree_dir / "inputs"
    inputs_dir.mkdir(parents=True, exist_ok=True)

    topic_by_camera = {camera_id: f"/cam{index}/image_raw" for index, camera_id in enumerate(camera_ids)}
    images_by_camera = {
        camera_id: [record for record in image_records if record.get("camera_id") == camera_id or record.get("topic") == topic]
        for camera_id, topic in topic_by_camera.items()
    }
    if any(not records for records in images_by_camera.values()):
        missing = [camera_id for camera_id, records in images_by_camera.items() if not records]
        raise RuntimeError(f"No ROS-free image records for cameras: {missing}")

    if len(camera_ids) > 1:
        images_by_camera = _align_camera_frames(images_by_camera, camera_ids)

    if image_stride > 1:
        for camera_id in camera_ids:
            images_by_camera[camera_id] = [
                record for index, record in enumerate(images_by_camera[camera_id]) if index % image_stride == 0
            ]

    selected_image_records = [record for records in images_by_camera.values() for record in records]
    imu_records = _crop_imu_for_selected_images(
        all_imu_records=all_imu_records,
        image_records=selected_image_records,
        imu_preroll_s=imu_preroll_s,
        imu_postroll_s=0.1,
    )
    if not imu_records:
        raise RuntimeError("No IMU records overlap the ROS-free image export window")

    strided_image_count = sum(len(records) for records in images_by_camera.values())
    filter_summary = {
        **filter_summary,
        "image_stride": image_stride,
        "imu_preroll_s": imu_preroll_s,
        "imu_preroll_samples": sum(
            1 for record in imu_records if record["timestamp_monotonic_ns"] < min(r["timestamp_monotonic_ns"] for r in selected_image_records)
        ),
        "imu_postroll_s": 0.1,
        "output_counts": {
            **filter_summary.get("output_counts", {}),
            "images": strided_image_count,
            "imu_samples": len(imu_records),
        },
    }

    camera_inputs = {}
    for index, camera_id in enumerate(camera_ids):
        records = images_by_camera[camera_id]
        image_dir = _common_parent([Path(record["image_path"]) for record in records])
        csv_path = inputs_dir / f"cam{index}.csv"
        with csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            for record in records:
                image_path = Path(record["image_path"])
                writer.writerow([_seconds_from_ns(record["timestamp_monotonic_ns"]), image_path.relative_to(image_dir)])
        camera_inputs[f"cam{index}"] = {
            "camera_id": camera_id,
            "csv": str(csv_path),
            "dir": str(image_dir),
            "frames": len(records),
        }

    imu_csv = inputs_dir / "imu.csv"
    with imu_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        for record in imu_records:
            gyro = record["gyro_radps"]
            accel = record["accel_mps2"]
            writer.writerow([_seconds_from_ns(record["timestamp_monotonic_ns"]), *gyro, *accel])

    summary = {
        "output_dir": str(rosfree_dir),
        "inputs_dir": str(inputs_dir),
        "camera_inputs": camera_inputs,
        "imu_csv": str(imu_csv),
        "counts": {
            "images": sum(item["frames"] for item in camera_inputs.values()),
            "imu_samples": len(imu_records),
        },
        "filters": filter_summary,
        "timestamp_basis": "timestamp_monotonic_ns",
    }
    (rosfree_dir / "rosfree_inputs_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _run_openvins_rosfree(*, prepared_dir: Path, runner_path: Path, stereo: bool) -> dict:
    rosfree_dir = prepared_dir / "rosfree"
    inputs_summary = json.loads((rosfree_dir / "rosfree_inputs_summary.json").read_text(encoding="utf-8"))
    runner_path = runner_path.resolve()
    if not runner_path.exists():
        raise FileNotFoundError(f"ROS-free OpenVINS runner does not exist: {runner_path}")
    if not os.access(runner_path, os.X_OK):
        raise PermissionError(
            "ROS-free OpenVINS runner is not executable: "
            f"{runner_path}\nFix: chmod +x {runner_path}"
        )

    trajectory_csv = rosfree_dir / "openvins_trajectory.csv"
    cmd = [
        str(runner_path),
        "--config",
        str(prepared_dir / "config" / "estimator_config.yaml"),
        "--imu",
        inputs_summary["imu_csv"],
        "--cam0-csv",
        inputs_summary["camera_inputs"]["cam0"]["csv"],
        "--cam0-dir",
        inputs_summary["camera_inputs"]["cam0"]["dir"],
        "--output",
        str(trajectory_csv),
    ]
    if stereo:
        cmd.append("--stereo")
    camera_inputs = inputs_summary["camera_inputs"]
    for index in range(1, len(camera_inputs)):
        key = f"cam{index}"
        cmd.extend(
            [
                f"--cam{index}-csv",
                camera_inputs[key]["csv"],
                f"--cam{index}-dir",
                camera_inputs[key]["dir"],
            ]
        )

    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    head_pose_path = prepared_dir / "head_pose.jsonl"
    conversion_summary = None
    if result.returncode == 0:
        conversion_summary = _write_head_pose_jsonl(trajectory_csv, head_pose_path, prepared_dir / "imu.jsonl")

    summary = {
        "ok": result.returncode == 0 and bool(conversion_summary and conversion_summary["poses"] > 0),
        "skipped": False,
        "command": cmd,
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
        "trajectory_csv": str(trajectory_csv),
        "head_pose": str(head_pose_path),
        "conversion": conversion_summary,
    }
    if result.returncode == 0 and conversion_summary is not None and conversion_summary["poses"] == 0:
        summary["hint"] = (
            "OpenVINS exited normally but wrote no states. Check initialization logs for static-init/disparity failures, "
            "then try a window with 2-3s stationary start followed by excitation, or revisit camera/IMU calibration."
        )
    if result.returncode != 0 and len(camera_inputs) > 2 and "Usage:" in result.stderr and "--cam2" not in result.stderr:
        summary["hint"] = (
            "The selected run_csv_msckf binary only advertises mono/stereo inputs. Rebuild it from the multi-camera "
            "run_csv_msckf.cpp source, or rerun with two --camera-id values while debugging the runner."
        )
    (rosfree_dir / "rosfree_run_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _restore_imu_preroll(
    *,
    all_imu_records: list[dict],
    filtered_imu_records: list[dict],
    image_records: list[dict],
    imu_preroll_s: float,
) -> list[dict]:
    if not image_records or not all_imu_records or imu_preroll_s <= 0:
        return filtered_imu_records
    first_image_ns = min(int(record["timestamp_unix_ns"]) for record in image_records)
    first_filtered_imu_ns = (
        min(int(record["timestamp_unix_ns"]) for record in filtered_imu_records)
        if filtered_imu_records
        else first_image_ns
    )
    preroll_start_ns = first_image_ns - int(imu_preroll_s * 1_000_000_000)
    records_by_time = {int(record["timestamp_unix_ns"]): record for record in filtered_imu_records}
    for record in all_imu_records:
        timestamp_ns = int(record["timestamp_unix_ns"])
        if preroll_start_ns <= timestamp_ns < first_filtered_imu_ns:
            records_by_time[timestamp_ns] = record
    return [records_by_time[timestamp] for timestamp in sorted(records_by_time)]


def _write_head_pose_jsonl(trajectory_csv: Path, head_pose_path: Path, imu_jsonl: Path) -> dict:
    if not trajectory_csv.exists():
        raise FileNotFoundError(f"Missing ROS-free trajectory CSV: {trajectory_csv}")
    offset_ns = _unix_minus_monotonic_offset_ns(imu_jsonl)
    written = 0
    with trajectory_csv.open("r", encoding="utf-8", newline="") as src, head_pose_path.open("w", encoding="utf-8") as dst:
        reader = csv.DictReader(src)
        for row in reader:
            timestamp_monotonic_ns = int(round(float(row["timestamp"]) * 1_000_000_000))
            timestamp_unix_ns = timestamp_monotonic_ns + offset_ns if offset_ns is not None else 0
            qx = float(row.get("q_x", 0.0))
            qy = float(row.get("q_y", 0.0))
            qz = float(row.get("q_z", 0.0))
            qw = float(row.get("q_w", 1.0))
            record = {
                "timestamp_unix_ns": timestamp_unix_ns,
                "timestamp_monotonic_ns": timestamp_monotonic_ns,
                "timestamp_source": "openvins_rosfree_csv",
                "tracking_state": 1,
                "T_W_H": {
                    "position": [float(row["p_x"]), float(row["p_y"]), float(row["p_z"])],
                    "orientation_wxyz": [qw, qx, qy, qz],
                },
            }
            dst.write(json.dumps(record, separators=(",", ":")) + "\n")
            written += 1
    return {
        "poses": written,
        "timestamp_unix_offset_ns": offset_ns,
    }


def _align_camera_frames(images_by_camera: dict[str, list[dict]], camera_ids: list[str]) -> dict[str, list[dict]]:
    records_by_group = {
        camera_id: {record.get("group_id"): record for record in images_by_camera[camera_id] if record.get("group_id") is not None}
        for camera_id in camera_ids
    }
    common_groups = sorted(set.intersection(*(set(records_by_group[camera_id]) for camera_id in camera_ids)))
    if common_groups:
        return {camera_id: [records_by_group[camera_id][group] for group in common_groups] for camera_id in camera_ids}

    count = min(len(images_by_camera[camera_id]) for camera_id in camera_ids)
    return {camera_id: images_by_camera[camera_id][:count] for camera_id in camera_ids}


def _common_parent(paths: list[Path]) -> Path:
    parents = {path.parent.resolve() for path in paths}
    if len(parents) == 1:
        return next(iter(parents))
    return Path(os.path.commonpath([str(path.resolve()) for path in parents]))


def _unix_minus_monotonic_offset_ns(imu_jsonl: Path) -> int | None:
    offsets = []
    for record in _read_jsonl(imu_jsonl):
        if record.get("timestamp_unix_ns") is None or record.get("timestamp_monotonic_ns") is None:
            continue
        offsets.append(int(record["timestamp_unix_ns"]) - int(record["timestamp_monotonic_ns"]))
        if len(offsets) >= 100:
            break
    if not offsets:
        return None
    offsets.sort()
    return offsets[len(offsets) // 2]


def _seconds_from_ns(value: int | str) -> str:
    return f"{int(value) / 1_000_000_000.0:.9f}"


def _read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def _crop_imu_for_selected_images(
    *,
    all_imu_records: list[dict],
    image_records: list[dict],
    imu_preroll_s: float,
    imu_postroll_s: float,
) -> list[dict]:
    if not all_imu_records or not image_records:
        return []
    image_times = [int(record["timestamp_monotonic_ns"]) for record in image_records]
    start_ns = min(image_times) - int(imu_preroll_s * 1e9)
    end_ns = max(image_times) + int(imu_postroll_s * 1e9)
    return [
        record
        for record in all_imu_records
        if start_ns <= int(record["timestamp_monotonic_ns"]) <= end_ns
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Run head OpenVINS through the ROS-free CSV/image runner.")
    parser.add_argument("--session-dir", required=True, help="Dashboard raw session, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--output-dir", help="Default: data/processed/<session_name>/openvins_head.")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--template-config-dir", default="open_vins/config/euroc_mav")
    parser.add_argument("--kalibr-imucam", help="Optional Kalibr camchain-imucam YAML with T_cam_imu/T_imu_cam and timeshift_cam_imu.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        help="Camera ID to include. Repeat for mono/stereo/multi-camera. Default: C1,C2,C0,C3.",
    )
    parser.add_argument("--imu-slot", default="head_imu")
    parser.add_argument("--runner", default=str(DEFAULT_ROSFREE_RUNNER), help="Path to run_csv_msckf.")
    parser.add_argument("--allow-not-ready", action="store_true", help="Write intermediate outputs even if P3a readiness checks fail.")
    parser.add_argument("--init-imu-thresh", type=float, default=0.5, help="OpenVINS static initializer IMU excitation threshold. Default: 0.5.")
    parser.add_argument("--init-max-disparity", type=float, default=10.0, help="OpenVINS static initializer max image disparity. Default: 10.")
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--timeshift-cam-imu", type=float, help="Static camera-to-IMU offset in seconds. OpenVINS uses imu_time = camera_time + offset.")
    parser.add_argument("--calib-cam-timeoffset", action="store_true", help="Let OpenVINS estimate camera-IMU time offset online.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Let OpenVINS estimate camera-IMU extrinsics online.")
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--prepare-only", action="store_true", help="Prepare CSV/image inputs but do not run OpenVINS.")
    parser.add_argument("--max-duration-s", type=float, help="Optional debug duration in seconds.")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Skip this many seconds from the beginning.")
    parser.add_argument("--imu-preroll-s", type=float, default=2.0, help="Keep this many seconds of IMU before the first exported image.")
    parser.add_argument("--image-stride", type=int, default=1, help="Write every Nth image while keeping all IMU samples.")
    args = parser.parse_args()

    try:
        summary = process_head_vio_rosfree(
            session_dir=Path(args.session_dir),
            output_dir=Path(args.output_dir) if args.output_dir else None,
            cameras_path=Path(args.cameras),
            template_config_dir=Path(args.template_config_dir) if args.template_config_dir else None,
            kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
            camera_ids=args.camera_ids,
            imu_slot=args.imu_slot,
            runner_path=Path(args.runner),
            fail_on_not_ready=not args.allow_not_ready,
            init_imu_thresh=args.init_imu_thresh,
            init_max_disparity=args.init_max_disparity,
            init_dyn_use=args.init_dyn_use,
            timeshift_cam_imu=args.timeshift_cam_imu,
            calib_cam_timeoffset=args.calib_cam_timeoffset,
            calib_cam_extrinsics=args.calib_cam_extrinsics,
            imu_time_mode=args.imu_time_mode,
            imu_rate_hz=args.imu_rate_hz,
            max_duration_s=args.max_duration_s,
            start_offset_s=args.start_offset_s,
            imu_preroll_s=args.imu_preroll_s,
            image_stride=args.image_stride,
            run_openvins=not args.prepare_only,
        )
    except PermissionError as exc:
        parser.exit(1, f"{exc}\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not summary["ok"] and not args.prepare_only:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
