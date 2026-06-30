from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from packages.head_vio_bridge.rosfree_runner import DEFAULT_ROSFREE_RUNNER, process_head_vio_rosfree


def sweep_openvins_timeshift(
    *,
    session_dir: Path,
    output_root: Path | None = None,
    camera_ids: list[str] | None = None,
    cameras_path: Path = Path("configs/cameras.yaml"),
    kalibr_imucam_path: Path | None = None,
    runner_path: Path = DEFAULT_ROSFREE_RUNNER,
    timeshift_ms_values: list[float] | None = None,
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    start_offset_s: float = 0.0,
    max_duration_s: float | None = None,
    imu_preroll_s: float = 2.0,
    image_stride: int = 1,
    init_imu_thresh: float = 0.5,
    init_max_disparity: float = 10.0,
    init_dyn_use: bool = False,
    calib_cam_timeoffset: bool = True,
    calib_cam_extrinsics: bool = False,
    fail_on_not_ready: bool = True,
    stop_on_success: bool = True,
) -> dict:
    """Run the ROS-free OpenVINS path over a grid of OpenVINS timeshift initial values."""

    session_dir = session_dir.resolve()
    output_root = (output_root or Path("data/processed") / session_dir.name / "openvins_timeshift_sweep").resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    offsets_ms = timeshift_ms_values or list(_float_range(-80.0, 80.0, 10.0))

    results = []
    for offset_ms in offsets_ms:
        label = _offset_label(offset_ms)
        run_dir = output_root / label
        try:
            summary = process_head_vio_rosfree(
                session_dir=session_dir,
                output_dir=run_dir,
                cameras_path=cameras_path,
                kalibr_imucam_path=kalibr_imucam_path,
                camera_ids=camera_ids,
                runner_path=runner_path,
                fail_on_not_ready=fail_on_not_ready,
                init_imu_thresh=init_imu_thresh,
                init_max_disparity=init_max_disparity,
                init_dyn_use=init_dyn_use,
                timeshift_cam_imu=offset_ms / 1000.0,
                calib_cam_timeoffset=calib_cam_timeoffset,
                calib_cam_extrinsics=calib_cam_extrinsics,
                imu_time_mode=imu_time_mode,
                imu_rate_hz=imu_rate_hz,
                max_duration_s=max_duration_s,
                start_offset_s=start_offset_s,
                imu_preroll_s=imu_preroll_s,
                image_stride=image_stride,
            )
            run_summary = summary.get("steps", {}).get("rosfree_run", {})
            conversion = run_summary.get("conversion") or {}
            result = {
                "timeshift_ms": offset_ms,
                "ok": bool(summary.get("ok")),
                "ready_for_p3a": bool(summary.get("ready_for_p3a")),
                "poses": int(conversion.get("poses") or 0),
                "output_dir": str(run_dir),
                "head_pose": summary.get("outputs", {}).get("head_pose"),
                "trajectory_csv": summary.get("outputs", {}).get("trajectory_csv"),
                "returncode": run_summary.get("returncode"),
                "init_tail": _extract_init_tail(run_summary.get("stdout_tail", "")),
            }
        except Exception as exc:  # keep the sweep going; one bad offset should not hide the others.
            result = {
                "timeshift_ms": offset_ms,
                "ok": False,
                "ready_for_p3a": False,
                "poses": 0,
                "output_dir": str(run_dir),
                "error": str(exc),
            }
        results.append(result)
        if stop_on_success and result["poses"] > 0:
            break

    best = max(results, key=lambda item: (int(item.get("poses") or 0), bool(item.get("ok"))), default=None)
    summary = {
        "session_dir": str(session_dir),
        "output_root": str(output_root),
        "camera_ids": camera_ids,
        "imu_time_mode": imu_time_mode,
        "init_imu_thresh": init_imu_thresh,
        "init_max_disparity": init_max_disparity,
        "init_dyn_use": init_dyn_use,
        "calib_cam_timeoffset": calib_cam_timeoffset,
        "calib_cam_extrinsics": calib_cam_extrinsics,
        "best": best,
        "results": results,
        "note": (
            "This uses OpenVINS' native timeshift_cam_imu initial value. Online calibration can refine it only after "
            "OpenVINS has initialized; it cannot recover IMU samples that were dropped before initialization."
        ),
    }
    (output_root / "timeshift_sweep_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def _float_range(start: float, stop: float, step: float):
    if step == 0:
        raise ValueError("step must be non-zero")
    value = start
    if step > 0:
        while value <= stop + 1e-9:
            yield round(value, 9)
            value += step
    else:
        while value >= stop - 1e-9:
            yield round(value, 9)
            value += step


def _offset_label(offset_ms: float) -> str:
    prefix = "p" if offset_ms >= 0 else "m"
    magnitude = abs(offset_ms)
    if abs(magnitude - round(magnitude)) < 1e-6:
        text = f"{int(round(magnitude)):03d}"
    else:
        text = f"{magnitude:06.2f}".replace(".", "p")
    return f"timeshift_{prefix}{text}ms"


def _extract_init_tail(text: str, *, max_lines: int = 8) -> list[str]:
    ansi = re.compile(r"\x1b\[[0-9;]*m")
    lines = []
    for line in text.splitlines():
        clean = ansi.sub("", line).strip()
        if "[init]" in clean or "camera-imu timeoffset" in clean:
            lines.append(clean)
    return lines[-max_lines:]


def _parse_offsets(args: argparse.Namespace) -> list[float]:
    if args.timeshift_ms:
        return [float(value) for value in args.timeshift_ms]
    return list(_float_range(args.timeshift_start_ms, args.timeshift_stop_ms, args.timeshift_step_ms))


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep OpenVINS timeshift_cam_imu initial values for head VIO debugging.")
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--output-root")
    parser.add_argument("--cameras", default="configs/cameras.yaml")
    parser.add_argument("--kalibr-imucam")
    parser.add_argument("--camera-id", action="append", dest="camera_ids", help="Repeat for LF/RF/etc. Default: OpenVINS head order.")
    parser.add_argument("--runner", default=str(DEFAULT_ROSFREE_RUNNER))
    parser.add_argument("--timeshift-ms", action="append", help="Explicit offset in milliseconds. Repeat to try multiple values.")
    parser.add_argument("--timeshift-start-ms", type=float, default=-80.0)
    parser.add_argument("--timeshift-stop-ms", type=float, default=80.0)
    parser.add_argument("--timeshift-step-ms", type=float, default=10.0)
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--max-duration-s", type=float)
    parser.add_argument("--imu-preroll-s", type=float, default=2.0)
    parser.add_argument("--image-stride", type=int, default=1)
    parser.add_argument("--init-imu-thresh", type=float, default=0.5)
    parser.add_argument("--init-max-disparity", type=float, default=10.0)
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--no-online-timeoffset", action="store_true", help="Do not set calib_cam_timeoffset=true during each run.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Also let OpenVINS estimate camera-IMU extrinsics after init.")
    parser.add_argument("--allow-not-ready", action="store_true")
    parser.add_argument("--no-stop-on-success", action="store_true")
    args = parser.parse_args()

    summary = sweep_openvins_timeshift(
        session_dir=Path(args.session_dir),
        output_root=Path(args.output_root) if args.output_root else None,
        camera_ids=args.camera_ids,
        cameras_path=Path(args.cameras),
        kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
        runner_path=Path(args.runner),
        timeshift_ms_values=_parse_offsets(args),
        imu_time_mode=args.imu_time_mode,
        imu_rate_hz=args.imu_rate_hz,
        start_offset_s=args.start_offset_s,
        max_duration_s=args.max_duration_s,
        imu_preroll_s=args.imu_preroll_s,
        image_stride=args.image_stride,
        init_imu_thresh=args.init_imu_thresh,
        init_max_disparity=args.init_max_disparity,
        init_dyn_use=args.init_dyn_use,
        calib_cam_timeoffset=not args.no_online_timeoffset,
        calib_cam_extrinsics=args.calib_cam_extrinsics,
        fail_on_not_ready=not args.allow_not_ready,
        stop_on_success=not args.no_stop_on_success,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not summary["best"] or int(summary["best"].get("poses") or 0) <= 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
