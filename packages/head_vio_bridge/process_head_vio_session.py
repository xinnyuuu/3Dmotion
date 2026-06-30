from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from packages.head_vio_bridge.openvins_config import DEFAULT_HEAD_CAMERA_IDS
from packages.head_vio_bridge.openvins_session import write_openvins_rosbag2
from packages.head_vio_bridge.p3_head_vio import prepare_p3_head_vio


def process_head_vio_session(
    session_dir: Path,
    *,
    output_dir: Path | None = None,
    cameras_path: Path = Path("configs/cameras.yaml"),
    template_config_dir: Path | None = Path("open_vins/config/euroc_mav"),
    kalibr_imucam_path: Path | None = None,
    camera_id: str | None = None,
    camera_ids: list[str] | None = None,
    imu_slot: str = "head_imu",
    write_rosbag: bool = True,
    overwrite_rosbag: bool = True,
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
    image_stride: int = 1,
) -> dict:
    """Prepare one raw dashboard session for OpenVINS and optional ROS2 replay.

    This is the one-command path for repeated P3a experiments:
    raw dashboard session -> readiness report -> OpenVINS JSONL/config -> rosbag2.
    """

    session_dir = session_dir.resolve()
    selected_camera_ids = camera_ids or ([camera_id] if camera_id else list(DEFAULT_HEAD_CAMERA_IDS))
    output_dir = output_dir or Path("data/processed") / session_dir.name / "openvins_head"
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    export_window = start_offset_s > 0 or max_duration_s is not None

    p3_summary = prepare_p3_head_vio(
        session_dir=session_dir,
        output_dir=output_dir,
        config_dir=output_dir / "config",
        cameras_path=cameras_path,
        template_config_dir=template_config_dir,
        kalibr_imucam_path=kalibr_imucam_path,
        camera_id=camera_id,
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
    )

    rosbag_summary = _maybe_write_rosbag2(
        output_dir=output_dir,
        imu_slot=imu_slot,
        enabled=write_rosbag and bool(p3_summary.get("ready_for_p3a")),
        overwrite=overwrite_rosbag,
        skipped_reason=None if write_rosbag else "Disabled by --no-rosbag2.",
        max_duration_s=None if export_window else max_duration_s,
        start_offset_s=0.0 if export_window else start_offset_s,
        image_stride=image_stride,
    )

    summary = {
        "session_dir": str(session_dir),
        "output_dir": str(output_dir),
        "ready_for_p3a": bool(p3_summary.get("ready_for_p3a")),
        "ok": bool(p3_summary.get("ready_for_p3a")) and (not write_rosbag or rosbag_summary.get("ok", False)),
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
            "rosbag2": rosbag_summary,
        },
        "next_commands": _next_commands(output_dir, camera_count=len(selected_camera_ids)),
        "notes": [
            "Head VIO exports the selected head cameras + head_imu. Wrist IMU and AprilTag are fused later.",
            "Run this command from a ROS2-sourced terminal if you want rosbag2 to be written in the same step.",
            "RViz shows head_imu as the temporary head frame: H := I_H.",
        ],
    }
    summary_path = output_dir / "head_vio_process_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _maybe_write_rosbag2(
    *,
    output_dir: Path,
    imu_slot: str,
    enabled: bool,
    overwrite: bool,
    skipped_reason: str | None,
    max_duration_s: float | None,
    start_offset_s: float,
    image_stride: int,
) -> dict:
    bag_dir = output_dir / "rosbag2"
    if not enabled:
        return {
            "ok": False,
            "skipped": True,
            "bag_dir": str(bag_dir),
            "reason": skipped_reason or "P3a readiness failed; rosbag2 export was skipped.",
        }

    if overwrite:
        _remove_existing_rosbag(output_dir=output_dir, bag_dir=bag_dir)

    try:
        rosbag_summary = write_openvins_rosbag2(
            prepared_dir=output_dir,
            bag_dir=bag_dir,
            frame_id=imu_slot,
            max_duration_s=max_duration_s,
            start_offset_s=start_offset_s,
            image_stride=image_stride,
        )
    except Exception as exc:
        return {
            "ok": False,
            "skipped": False,
            "bag_dir": str(bag_dir),
            "error": str(exc),
            "hint": (
                "If this says ROS2 Python packages are missing, run: "
                "`source /opt/ros/humble/setup.bash`, then rerun this same process command."
            ),
        }

    return {
        "ok": True,
        "skipped": False,
        "bag_dir": str(bag_dir),
        "summary": rosbag_summary,
    }


def _remove_existing_rosbag(*, output_dir: Path, bag_dir: Path) -> None:
    output_root = output_dir.resolve()
    bag_root = bag_dir.resolve()
    if bag_root.name != "rosbag2" or not bag_root.is_relative_to(output_root):
        raise ValueError(f"Refusing to overwrite unexpected rosbag2 path: {bag_dir}")
    if bag_root.exists():
        shutil.rmtree(bag_root)
    summary_path = bag_root.with_name(f"{bag_root.name}_summary.json")
    if summary_path.exists():
        summary_path.unlink()


def _next_commands(output_dir: Path, *, camera_count: int) -> dict:
    config_path = output_dir / "config" / "estimator_config.yaml"
    bag_dir = output_dir / "rosbag2"
    return {
        "run_openvins": (
            "source scripts/source_openvins_ros2.bash\n"
            "ros2 launch ov_msckf subscribe.launch.py "
            f"config_path:={config_path} "
            f"max_cameras:={camera_count} use_stereo:=false"
        ),
        "play_rosbag2": (
            "source /opt/ros/humble/setup.bash\n"
            f"ros2 bag play {bag_dir}"
        ),
        "replay_without_rosbag2": (
            f"scripts/session_replay_publisher_ros2.bash --prepared-dir {output_dir}"
        ),
        "launch_rviz": (
            "cd ros2_ws\n"
            "source /opt/ros/humble/setup.bash\n"
            "source install/setup.bash\n"
            "ros2 launch vimas_motion_bringup head_vio_rviz.launch.py"
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="One-command P3a processing: raw session -> OpenVINS inputs/config -> optional rosbag2."
    )
    parser.add_argument("--session-dir", required=True, help="Dashboard raw session, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--output-dir", help="Default: data/processed/<session_name>/openvins_head.")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--template-config-dir", default="open_vins/config/euroc_mav")
    parser.add_argument("--kalibr-imucam", help="Optional Kalibr camchain-imucam YAML with T_cam_imu/T_imu_cam and timeshift_cam_imu.")
    parser.add_argument(
        "--camera-id",
        action="append",
        dest="camera_ids",
        help="Camera ID to include. Repeat for multiple cameras. Default: C1,C2,C0,C3.",
    )
    parser.add_argument("--imu-slot", default="head_imu")
    parser.add_argument("--no-rosbag2", action="store_true", help="Only write JSONL/config; skip rosbag2 export.")
    parser.add_argument("--keep-existing-rosbag2", action="store_true", help="Do not delete an existing rosbag2 directory before export.")
    parser.add_argument("--allow-not-ready", action="store_true", help="Write intermediate outputs even if P3a readiness checks fail.")
    parser.add_argument("--init-imu-thresh", type=float, default=0.5, help="OpenVINS static initializer IMU excitation threshold. Default: 0.5.")
    parser.add_argument("--init-max-disparity", type=float, default=10.0, help="OpenVINS static initializer max image disparity. Default: 10.")
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--timeshift-cam-imu", type=float, help="Static camera-to-IMU offset in seconds. OpenVINS uses imu_time = camera_time + offset.")
    parser.add_argument("--calib-cam-timeoffset", action="store_true", help="Let OpenVINS estimate camera-IMU time offset online.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Let OpenVINS estimate camera-IMU extrinsics online.")
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--max-duration-s", type=float, help="Optional debug rosbag2 export duration in seconds.")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Skip this many seconds from the beginning before writing rosbag2.")
    parser.add_argument("--image-stride", type=int, default=1, help="Write every Nth image to rosbag2 while keeping all IMU samples.")
    args = parser.parse_args()

    summary = process_head_vio_session(
        session_dir=Path(args.session_dir),
        output_dir=Path(args.output_dir) if args.output_dir else None,
        cameras_path=Path(args.cameras),
        template_config_dir=Path(args.template_config_dir) if args.template_config_dir else None,
        kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
        camera_ids=args.camera_ids,
        imu_slot=args.imu_slot,
        write_rosbag=not args.no_rosbag2,
        overwrite_rosbag=not args.keep_existing_rosbag2,
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
        image_stride=args.image_stride,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if not summary["ready_for_p3a"] and not args.allow_not_ready:
        raise SystemExit(1)
    if not args.no_rosbag2 and not summary["steps"]["rosbag2"].get("ok", False):
        raise SystemExit(2)
    raise SystemExit(0)


if __name__ == "__main__":
    main()
