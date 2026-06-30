from __future__ import annotations

import argparse
import json
from pathlib import Path

from packages.apriltag_ring_node.process_session import process_session as process_apriltag_session
from packages.apriltag_ring_node.world_anchor import process_world_anchor_session
from packages.head_vio_bridge.openvins_config import generate_openvins_config
from packages.head_vio_bridge.openvins_session import prepare_openvins_session
from packages.head_vio_bridge.rosfree_runner import DEFAULT_ROSFREE_RUNNER, process_head_vio_rosfree
from packages.session_tools.motion_fusion import fuse_motion_session
from packages.session_tools.validate_session import validate_session


def postprocess_session(
    session_dir: Path,
    output_root: Path | None,
    cameras_path: Path,
    bracelet_path: Path,
    world_tags_path: Path = Path("configs/world_tags.yaml"),
    kalibr_imucam_path: Path | None = None,
    camera_ids: list[str] | None = None,
    run_apriltag: bool = False,
    run_world_anchor: bool = False,
    run_openvins: bool = False,
    run_head_vio_rosfree: bool = False,
    generate_config: bool = False,
    fuse_motion: bool = False,
    head_pose_path: Path | None = None,
    rosfree_runner: Path = DEFAULT_ROSFREE_RUNNER,
    max_duration_s: float | None = None,
    start_offset_s: float = 0.0,
    imu_preroll_s: float = 2.0,
    image_stride: int = 1,
    init_imu_thresh: float = 0.5,
    init_max_disparity: float = 10.0,
    init_dyn_use: bool = False,
    timeshift_cam_imu: float | None = None,
    calib_cam_timeoffset: bool = False,
    calib_cam_extrinsics: bool = False,
    imu_time_mode: str = "raw",
    imu_rate_hz: float = 200.0,
    enable_hands: bool = False,
    max_hands: int = 2,
    max_hand_skeletons_per_frame: int = 1,
    hand_multiview_only: bool = False,
    hand_allow_direct_singleview_fallback: bool = False,
    hand_detection_confidence: float = 0.5,
    hand_tracking_confidence: float = 0.5,
    hand_model: Path | None = None,
    wrist_position_visual_alpha: float = 0.75,
    wrist_visual_correction_alpha: float = 0.35,
    wrist_prediction_gate_translation_m: float = 0.20,
    wrist_prediction_gate_rotation_deg: float = 60.0,
    wrist_static_gyro_thresh_radps: float = 0.18,
    wrist_static_accel_std_thresh_mps2: float = 0.25,
    head_position_visual_alpha: float = 0.75,
    head_orientation_visual_alpha: float = 0.12,
    max_position_prediction_gap_s: float = 0.25,
    multi_camera_gate_translation_m: float = 0.12,
    multi_camera_gate_rotation_deg: float = 25.0,
    multi_camera_median_gate_m: float = 0.08,
) -> dict:
    session_dir = session_dir.resolve()
    if output_root is None:
        output_root = Path("data/processed") / session_dir.name
    output_root.mkdir(parents=True, exist_ok=True)
    summary = {
        "session_dir": str(session_dir),
        "output_root": str(output_root.resolve()),
        "validation": validate_session(session_dir),
        "steps": {},
    }
    if not summary["validation"]["ok_for_camera_replay"]:
        summary["steps"]["skipped"] = "Camera replay is not ready. Fix capture before running AprilTag/OpenVINS processing."
        _write_summary(output_root, summary)
        return summary

    if run_apriltag:
        apriltag_dir = output_root / "wrist_visual"
        process_apriltag_session(
            session_dir=session_dir / "cameras",
            cameras_path=cameras_path,
            bracelet_path=bracelet_path,
            output_dir=apriltag_dir,
        )
        summary["steps"]["apriltag"] = {
            "output_dir": str(apriltag_dir),
            "wrist_visual_candidates": str(apriltag_dir / "wrist_visual_candidates.jsonl"),
            "wrist_visual_pose": str(apriltag_dir / "wrist_visual_pose.jsonl"),
            "legacy": True,
            "replacement": "Use --world-anchor for the AprilGrid world-frame visual workflow.",
        }

    if run_world_anchor:
        world_anchor_dir = output_root / "world_anchor"
        world_summary = process_world_anchor_session(
            session_dir=session_dir,
            cameras_path=cameras_path,
            world_tags_path=world_tags_path,
            bracelet_path=bracelet_path,
            output_dir=world_anchor_dir,
            enable_hands=enable_hands,
            max_hands=max_hands,
            max_hand_skeletons_per_frame=max_hand_skeletons_per_frame,
            hand_multiview_only=hand_multiview_only,
            hand_allow_direct_singleview_fallback=hand_allow_direct_singleview_fallback,
            hand_detection_confidence=hand_detection_confidence,
            hand_tracking_confidence=hand_tracking_confidence,
            hand_model=hand_model,
            head_position_visual_alpha=head_position_visual_alpha,
            head_orientation_visual_alpha=head_orientation_visual_alpha,
            max_position_prediction_gap_s=max_position_prediction_gap_s,
            wrist_position_visual_alpha=wrist_position_visual_alpha,
            wrist_visual_correction_alpha=wrist_visual_correction_alpha,
            wrist_prediction_gate_translation_m=wrist_prediction_gate_translation_m,
            wrist_prediction_gate_rotation_deg=wrist_prediction_gate_rotation_deg,
            wrist_static_gyro_thresh_radps=wrist_static_gyro_thresh_radps,
            wrist_static_accel_std_thresh_mps2=wrist_static_accel_std_thresh_mps2,
            multi_camera_gate_translation_m=multi_camera_gate_translation_m,
            multi_camera_gate_rotation_deg=multi_camera_gate_rotation_deg,
            multi_camera_median_gate_m=multi_camera_median_gate_m,
        )
        summary["steps"]["world_anchor"] = {
            "output_dir": str(world_anchor_dir),
            "world_anchor_candidates": world_summary["outputs"]["world_anchor_candidates"],
            "wrist_world_candidates": world_summary["outputs"]["wrist_world_candidates"],
            "head_pose": world_summary["outputs"]["head_pose"],
            "motion_frame": world_summary["outputs"]["motion_frame"],
            "hand_skeletons": world_summary["outputs"].get("hand_skeletons"),
            "world_tags": world_summary["world_tags"],
            "bracelet": world_summary["bracelet"],
        }

    if run_openvins:
        openvins_dir = output_root / "openvins_session"
        summary["steps"]["openvins_session"] = prepare_openvins_session(
            session_dir=session_dir,
            output_dir=openvins_dir,
            camera_ids=camera_ids,
            imu_slot="head_imu",
        )

    if run_head_vio_rosfree:
        summary["steps"]["head_vio_rosfree"] = process_head_vio_rosfree(
            session_dir=session_dir,
            output_dir=output_root / "openvins_head",
            cameras_path=cameras_path,
            kalibr_imucam_path=kalibr_imucam_path,
            camera_ids=camera_ids,
            runner_path=rosfree_runner,
            max_duration_s=max_duration_s,
            start_offset_s=start_offset_s,
            imu_preroll_s=imu_preroll_s,
            image_stride=image_stride,
            init_imu_thresh=init_imu_thresh,
            init_max_disparity=init_max_disparity,
            init_dyn_use=init_dyn_use,
            timeshift_cam_imu=timeshift_cam_imu,
            calib_cam_timeoffset=calib_cam_timeoffset,
            calib_cam_extrinsics=calib_cam_extrinsics,
            imu_time_mode=imu_time_mode,
            imu_rate_hz=imu_rate_hz,
        )
        if head_pose_path is None:
            head_pose_path = output_root / "openvins_head" / "head_pose.jsonl"

    if generate_config:
        config_dir = output_root / "openvins_config"
        summary["steps"]["openvins_config"] = generate_openvins_config(
            cameras_path=cameras_path,
            output_dir=config_dir,
            camera_ids=camera_ids,
            init_imu_thresh=init_imu_thresh,
            kalibr_imucam_path=kalibr_imucam_path,
            init_max_disparity=init_max_disparity,
            init_dyn_use=init_dyn_use,
            timeshift_cam_imu=timeshift_cam_imu,
            calib_cam_timeoffset=calib_cam_timeoffset,
            calib_cam_extrinsics=calib_cam_extrinsics,
        )

    if fuse_motion:
        summary["steps"]["motion_fusion"] = fuse_motion_session(
            session_dir=session_dir,
            output_root=output_root,
            head_pose_path=head_pose_path,
        )

    _write_summary(output_root, summary)
    return summary


def _write_summary(output_root: Path, summary: dict) -> None:
    (output_root / "postprocess_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and optionally postprocess one recorded 3DMotion session.")
    parser.add_argument("--session-dir", required=True, help="Dashboard session directory, e.g. data/raw/session_YYYYMMDD_HHMMSS.")
    parser.add_argument("--output-root", default=None, help="Output root for all generated files. Default: data/processed/<session_name>.")
    parser.add_argument("--cameras", default="configs/cameras.yaml", help="Camera calibration YAML.")
    parser.add_argument("--bracelet", default="configs/bracelet.yaml", help="Bracelet geometry YAML.")
    parser.add_argument("--world-tags", default="configs/world_tags.yaml", help="AprilGrid world-anchor YAML.")
    parser.add_argument("--kalibr-imucam", help="Optional Kalibr camchain-imucam YAML with T_cam_imu/T_imu_cam and timeshift_cam_imu.")
    parser.add_argument("--camera-id", action="append", dest="camera_ids", help="Camera ID for OpenVINS export/config. Repeat for multiple cameras. Default: C1,C2,C0,C3.")
    parser.add_argument("--apriltag", action="store_true", help="Run legacy offline wrist AprilTag processing into wrist_visual/T_H_B.")
    parser.add_argument("--world-anchor", action="store_true", help="Run AprilGrid world-anchor processing into world_anchor/motion_frame.jsonl.")
    parser.add_argument("--openvins", action="store_true", help="Prepare OpenVINS image/IMU JSONL streams.")
    parser.add_argument("--head-vio-rosfree", action="store_true", help="Run ROS-free OpenVINS head VIO and write openvins_head/head_pose.jsonl.")
    parser.add_argument("--openvins-config", action="store_true", help="Generate first-pass OpenVINS config files.")
    parser.add_argument("--fuse-motion", action="store_true", help="Fuse head_pose.jsonl, wrist_visual_pose.jsonl, and wrist_imu.jsonl.")
    parser.add_argument("--head-pose", help="Optional head_pose.jsonl path for --fuse-motion.")
    parser.add_argument("--rosfree-runner", default=str(DEFAULT_ROSFREE_RUNNER), help="Path to run_csv_msckf for --head-vio-rosfree.")
    parser.add_argument("--max-duration-s", type=float, help="Optional debug duration for --head-vio-rosfree.")
    parser.add_argument("--start-offset-s", type=float, default=0.0, help="Optional start offset for --head-vio-rosfree.")
    parser.add_argument("--imu-preroll-s", type=float, default=2.0, help="Keep this many seconds of IMU before the first exported image.")
    parser.add_argument("--image-stride", type=int, default=1, help="Image stride for --head-vio-rosfree.")
    parser.add_argument("--init-imu-thresh", type=float, default=0.5, help="OpenVINS static initializer IMU excitation threshold.")
    parser.add_argument("--init-max-disparity", type=float, default=10.0, help="OpenVINS static initializer max image disparity.")
    parser.add_argument("--init-dyn-use", action="store_true", help="Use OpenVINS dynamic initializer when the static init window is moving.")
    parser.add_argument("--timeshift-cam-imu", type=float, help="Static camera-to-IMU offset in seconds. OpenVINS uses imu_time = camera_time + offset.")
    parser.add_argument("--calib-cam-timeoffset", action="store_true", help="Let OpenVINS estimate camera-IMU time offset online.")
    parser.add_argument("--calib-cam-extrinsics", action="store_true", help="Let OpenVINS estimate camera-IMU extrinsics online.")
    parser.add_argument("--imu-time-mode", choices=["raw", "resample-rate", "reconstruct-rate"], default="raw")
    parser.add_argument("--imu-rate-hz", type=float, default=200.0)
    parser.add_argument("--hands", action="store_true", help="With --world-anchor, detect MediaPipe hand skeletons and add approximate world landmarks.")
    parser.add_argument("--max-hands", type=int, default=2)
    parser.add_argument("--max-hand-skeletons-per-frame", type=int, default=1)
    parser.add_argument("--hand-multiview-only", action="store_true", help="Only output hand skeletons reconstructed from at least two cameras.")
    parser.add_argument("--hand-allow-direct-singleview-fallback", action="store_true", help="Allow raw single-camera wrist-depth hand output when no recent multiview/guided hand exists.")
    parser.add_argument("--hand-detection-confidence", type=float, default=0.5)
    parser.add_argument("--hand-tracking-confidence", type=float, default=0.5)
    parser.add_argument("--hand-model", help="Optional MediaPipe Tasks hand_landmarker.task path.")
    parser.add_argument("--wrist-position-visual-alpha", type=float, default=0.75)
    parser.add_argument("--wrist-visual-correction-alpha", type=float, default=0.35)
    parser.add_argument("--wrist-prediction-gate-translation-m", type=float, default=0.20)
    parser.add_argument("--wrist-prediction-gate-rotation-deg", type=float, default=60.0)
    parser.add_argument("--wrist-static-gyro-thresh-radps", type=float, default=0.18)
    parser.add_argument("--wrist-static-accel-std-thresh-mps2", type=float, default=0.25)
    parser.add_argument("--head-position-visual-alpha", type=float, default=0.75)
    parser.add_argument("--head-orientation-visual-alpha", type=float, default=0.12)
    parser.add_argument("--max-position-prediction-gap-s", type=float, default=0.25)
    parser.add_argument("--multi-camera-gate-translation-m", type=float, default=0.12)
    parser.add_argument("--multi-camera-gate-rotation-deg", type=float, default=25.0)
    parser.add_argument("--multi-camera-median-gate-m", type=float, default=0.08)
    args = parser.parse_args()

    summary = postprocess_session(
        session_dir=Path(args.session_dir),
        output_root=Path(args.output_root) if args.output_root else None,
        cameras_path=Path(args.cameras),
        bracelet_path=Path(args.bracelet),
        world_tags_path=Path(args.world_tags),
        kalibr_imucam_path=Path(args.kalibr_imucam) if args.kalibr_imucam else None,
        camera_ids=args.camera_ids,
        run_apriltag=args.apriltag,
        run_world_anchor=args.world_anchor,
        run_openvins=args.openvins,
        run_head_vio_rosfree=args.head_vio_rosfree,
        generate_config=args.openvins_config,
        fuse_motion=args.fuse_motion,
        head_pose_path=Path(args.head_pose) if args.head_pose else None,
        rosfree_runner=Path(args.rosfree_runner),
        max_duration_s=args.max_duration_s,
        start_offset_s=args.start_offset_s,
        imu_preroll_s=args.imu_preroll_s,
        image_stride=args.image_stride,
        init_imu_thresh=args.init_imu_thresh,
        init_max_disparity=args.init_max_disparity,
        init_dyn_use=args.init_dyn_use,
        timeshift_cam_imu=args.timeshift_cam_imu,
        calib_cam_timeoffset=args.calib_cam_timeoffset,
        calib_cam_extrinsics=args.calib_cam_extrinsics,
        imu_time_mode=args.imu_time_mode,
        imu_rate_hz=args.imu_rate_hz,
        enable_hands=args.hands,
        max_hands=args.max_hands,
        max_hand_skeletons_per_frame=args.max_hand_skeletons_per_frame,
        hand_multiview_only=args.hand_multiview_only,
        hand_allow_direct_singleview_fallback=args.hand_allow_direct_singleview_fallback,
        hand_detection_confidence=args.hand_detection_confidence,
        hand_tracking_confidence=args.hand_tracking_confidence,
        hand_model=Path(args.hand_model) if args.hand_model else None,
        wrist_position_visual_alpha=args.wrist_position_visual_alpha,
        wrist_visual_correction_alpha=args.wrist_visual_correction_alpha,
        wrist_prediction_gate_translation_m=args.wrist_prediction_gate_translation_m,
        wrist_prediction_gate_rotation_deg=args.wrist_prediction_gate_rotation_deg,
        wrist_static_gyro_thresh_radps=args.wrist_static_gyro_thresh_radps,
        wrist_static_accel_std_thresh_mps2=args.wrist_static_accel_std_thresh_mps2,
        head_position_visual_alpha=args.head_position_visual_alpha,
        head_orientation_visual_alpha=args.head_orientation_visual_alpha,
        max_position_prediction_gap_s=args.max_position_prediction_gap_s,
        multi_camera_gate_translation_m=args.multi_camera_gate_translation_m,
        multi_camera_gate_rotation_deg=args.multi_camera_gate_rotation_deg,
        multi_camera_median_gate_m=args.multi_camera_median_gate_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
