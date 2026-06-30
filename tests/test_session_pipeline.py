from __future__ import annotations

import asyncio
import json
import math
import sys
import types
from pathlib import Path

from packages.head_vio_bridge.p3_head_vio import check_head_vio_readiness, prepare_p3_head_vio
from packages.head_vio_bridge.process_head_vio_session import process_head_vio_session
from packages.head_vio_bridge.rosfree_runner import process_head_vio_rosfree
from packages.head_vio_bridge.euroc_session import prepare_euroc_openvins_session
from packages.head_vio_bridge.openvins_config import generate_openvins_config
from packages.head_vio_bridge.openvins_session import prepare_openvins_session
from packages.head_vio_bridge.openvins_session import _filter_rosbag_records
from packages.apriltag_ring_node.config import load_camera_calibrations, load_camera_priority
from packages.capture_dashboard import server as dashboard_server
from packages.imu_ble_bridge import wt901
from packages.session_tools.motion_fusion import fuse_motion_session
from packages.session_tools.postprocess_session import postprocess_session
from packages.session_tools.validate_session import validate_session


def test_validate_session_allows_partial_camera_failures(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=True, with_head_imu=True)
    cameras = session / "cameras"
    _write_jsonl(cameras / "capture_errors.jsonl", [{"camera_id": "C1", "source": "/dev/video2", "error": "open_failed"}])

    summary = validate_session(session)

    assert summary["ok_for_camera_replay"] is True
    assert summary["has_capture_warnings"] is True
    assert summary["camera_frame_count"] == 1
    assert summary["camera_counts"] == {"C0": 1}
    assert summary["imu_counts"] == {"head_imu": 1}


def test_validate_session_reports_no_camera_frames(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=False, with_head_imu=False)
    _write_jsonl(session / "cameras" / "capture_errors.jsonl", [{"camera_id": "C0", "source": "/dev/video0", "error": "open_failed"}])

    summary = validate_session(session)

    assert summary["ok_for_camera_replay"] is False
    assert summary["camera_frame_count"] == 0
    assert "Camera opened failed" in summary["next_steps"][0]


def test_validate_session_rejects_missing_frame_images(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=True, with_head_imu=True)
    (session / "cameras" / "C0" / "00000000.jpg").unlink()

    summary = validate_session(session)

    assert summary["ok_for_camera_replay"] is False
    camera_frame_check = next(check for check in summary["checks"] if check["name"] == "camera_frames")
    assert "1 missing image files" in camera_frame_check["message"]


def test_prepare_openvins_session_exports_image_and_imu_streams(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=True, with_head_imu=True)
    output = tmp_path / "processed" / "openvins_session"

    summary = prepare_openvins_session(session, output, camera_ids=["C0"])

    assert summary["counts"] == {"images": 1, "imu_samples": 1}
    image_record = _read_jsonl(output / "images.jsonl")[0]
    imu_record = _read_jsonl(output / "imu.jsonl")[0]
    assert image_record["topic"] == "/cam0/image_raw"
    assert image_record["camera_id"] == "C0"
    assert image_record["image_path"].endswith("cameras/C0/00000000.jpg")
    assert imu_record["topic"] == "/imu0"
    assert imu_record["sensor_id"] == "head_imu"


def test_camera_config_preserves_omni_intrinsics() -> None:
    calibrations = load_camera_calibrations(Path("configs/cameras.yaml"))
    priority = load_camera_priority(Path("configs/cameras.yaml"))

    assert priority == ["C1", "C2", "C0", "C3"]
    assert calibrations["C1"].projection_model == "mei"
    assert calibrations["C1"].uses_omni_projection is True
    assert calibrations["C1"].xi == 1.773
    assert calibrations["C1"].intrinsics[0, 0] == 1628.9
    assert calibrations["C3"].distortion.reshape(-1).tolist()[:2] == [0.409, -1.119]


def test_openvins_config_exports_four_camera_compatibility_view(tmp_path: Path) -> None:
    output = tmp_path / "openvins_config"

    summary = generate_openvins_config(
        cameras_path=Path("configs/cameras.yaml"),
        output_dir=output,
        template_config_dir=None,
    )

    assert summary["camera_ids"] == ["C1", "C2", "C0", "C3"]
    assert "Mei/omni" in summary["projection_note"]
    estimator_text = (output / "estimator_config.yaml").read_text(encoding="utf-8")
    assert "max_cameras: 4" in estimator_text
    assert "calib_cam_extrinsics: false" in estimator_text
    assert "calib_cam_intrinsics: false" in estimator_text
    assert "calib_cam_timeoffset: false" in estimator_text
    imucam_text = (output / "kalibr_imucam_chain.yaml").read_text(encoding="utf-8")
    assert "timeshift_cam_imu: 0.0" in imucam_text
    assert "source_xi: 1.6213112201599278" in imucam_text
    assert "source_projection_model: mei" in imucam_text
    assert "camera_model: omni" in imucam_text
    assert "intrinsics: [1.6213112201599278, 1543.1254352628462, 1546.7811430542765, 843.507636283496, 633.2611142767973]" in imucam_text


def test_openvins_config_uses_imu_calibration(tmp_path: Path) -> None:
    output = tmp_path / "openvins_config"

    summary = generate_openvins_config(
        cameras_path=Path("configs/cameras.yaml"),
        output_dir=output,
        template_config_dir=None,
        imu_calibration_path=Path("configs/imu_calibration.yaml"),
    )

    assert summary["imu_calibration"]["head_imu_sample_count"] == 10240
    imu_text = (output / "kalibr_imu_chain.yaml").read_text(encoding="utf-8")
    assert "accelerometer_noise_density: 0.00038201490199235826" in imu_text
    assert "gyroscope_noise_density: 6.606039558454192e-06" in imu_text
    assert "source_imu_id: head_imu" in imu_text


def test_prepare_openvins_session_defaults_to_four_head_cameras(tmp_path: Path) -> None:
    session = _make_quad_session(tmp_path)
    output = tmp_path / "processed" / "openvins_session"

    summary = prepare_openvins_session(session, output)
    image_records = _read_jsonl(output / "images.jsonl")

    assert summary["openvins_topics"]["cameras"] == {
        "C1": "/cam0/image_raw",
        "C2": "/cam1/image_raw",
        "C0": "/cam2/image_raw",
        "C3": "/cam3/image_raw",
    }
    assert [record["camera_id"] for record in image_records[:4]] == ["C1", "C2", "C0", "C3"]


def test_postprocess_session_skips_when_camera_replay_not_ready(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=False, with_head_imu=True)
    output = tmp_path / "processed"

    summary = postprocess_session(
        session_dir=session,
        output_root=output,
        cameras_path=Path("configs/cameras.yaml"),
        bracelet_path=Path("configs/bracelet.yaml"),
        run_openvins=True,
        generate_config=True,
    )

    assert summary["validation"]["ok_for_camera_replay"] is False
    assert "skipped" in summary["steps"]
    assert (output / "postprocess_summary.json").exists()


def test_head_vio_readiness_accepts_c0_and_head_imu_session(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)

    report = check_head_vio_readiness(session, camera_id="C0", imu_slot="head_imu", min_duration_s=1.0)

    assert report["ready_for_p3a"] is True
    assert report["p3a_frame_convention"]["prototype_output"] == "T_W_H := T_W_I"
    assert all(check["ok"] for check in report["checks"])


def test_prepare_p3_head_vio_writes_session_and_config(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    output = tmp_path / "processed" / "openvins_c0"
    config = tmp_path / "processed" / "openvins_c0" / "config"

    summary = prepare_p3_head_vio(
        session_dir=session,
        output_dir=output,
        config_dir=config,
        cameras_path=Path("configs/cameras.yaml"),
        camera_id="C0",
        imu_slot="head_imu",
    )

    assert summary["ready_for_p3a"] is True
    assert (output / "images.jsonl").exists()
    assert (output / "imu.jsonl").exists()
    assert (output / "p3_head_vio_summary.json").exists()
    assert (config / "estimator_config.yaml").exists()
    assert (config / "kalibr_imucam_chain.yaml").exists()


def test_process_head_vio_session_writes_one_command_outputs_without_rosbag(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    output = tmp_path / "processed" / "openvins_c0"

    summary = process_head_vio_session(
        session_dir=session,
        output_dir=output,
        cameras_path=Path("configs/cameras.yaml"),
        camera_id="C0",
        imu_slot="head_imu",
        write_rosbag=False,
    )

    assert summary["ok"] is True
    assert summary["ready_for_p3a"] is True
    assert summary["steps"]["rosbag2"]["skipped"] is True
    assert (output / "images.jsonl").exists()
    assert (output / "imu.jsonl").exists()
    assert (output / "config" / "estimator_config.yaml").exists()
    assert (output / "head_vio_process_summary.json").exists()


def test_prepare_openvins_session_reconstructs_imu_time_by_sample_index(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    imu_path = session / "imus" / "head_imu.jsonl"
    records = _read_jsonl(imu_path)
    for index, record in enumerate(records):
        record["timestamp_monotonic_ns"] = 900_000_000 + index * (1_000 if index % 2 == 0 else 60_000_000)
        record["timestamp_unix_ns"] = record["timestamp_monotonic_ns"] + 100
    _write_jsonl(imu_path, records)
    output = tmp_path / "processed" / "openvins_reconstruct"

    summary = prepare_openvins_session(
        session,
        output,
        camera_ids=["C0"],
        imu_time_mode="reconstruct-rate",
        imu_rate_hz=200.0,
    )

    imu_records = _read_jsonl(output / "imu.jsonl")
    timestamps = [record["timestamp_monotonic_ns"] for record in imu_records[:4]]
    assert timestamps == [900_000_000, 905_000_000, 910_000_000, 915_000_000]
    assert imu_records[0]["timestamp_source"] == "offline_reconstructed_200hz_by_sample_index"
    assert summary["imu_time"]["exported"]["dt_ms_median"] == 5.0


def test_head_vio_readiness_rejects_large_imu_timestamp_gaps(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    imu_path = session / "imus" / "head_imu.jsonl"
    records = _read_jsonl(imu_path)
    for index, record in enumerate(records):
        record["timestamp_monotonic_ns"] = 900_000_000 + index * 60_000_000
        record["timestamp_unix_ns"] = record["timestamp_monotonic_ns"]
    _write_jsonl(imu_path, records)

    report = check_head_vio_readiness(
        session,
        camera_id="C0",
        min_accel_std_mps2=0.01,
        max_imu_dt_ms=50.0,
        max_imu_p99_dt_ms=20.0,
    )

    imu_check = next(check for check in report["checks"] if check["name"] == "imu_stream")
    assert report["ready_for_p3a"] is False
    assert imu_check["ok"] is False
    assert imu_check["details"]["dt_ms_max"] > 50.0


def test_head_vio_readiness_can_check_clean_window_after_startup_gap(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    imu_path = session / "imus" / "head_imu.jsonl"
    records = _startup_gap_then_clean_imu_records()
    _write_jsonl(imu_path, records)

    full_report = check_head_vio_readiness(
        session,
        camera_id="C0",
        min_duration_s=1.0,
        min_accel_std_mps2=0.01,
    )
    window_report = check_head_vio_readiness(
        session,
        camera_id="C0",
        min_duration_s=1.0,
        min_accel_std_mps2=0.01,
        start_offset_s=1.0,
        max_duration_s=4.0,
    )

    assert full_report["ready_for_p3a"] is False
    assert window_report["ready_for_p3a"] is True
    assert window_report["readiness_window"]["output_counts"]["imu_samples"] > 0


def test_prepare_openvins_session_reconstructs_time_after_window_crop(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    imu_path = session / "imus" / "head_imu.jsonl"
    _write_jsonl(imu_path, _startup_gap_then_clean_imu_records())
    output = tmp_path / "processed" / "openvins_window_reconstruct"

    summary = prepare_openvins_session(
        session,
        output,
        camera_ids=["C0"],
        imu_time_mode="reconstruct-rate",
        imu_rate_hz=200.0,
        export_window=True,
        export_start_offset_s=1.0,
        export_max_duration_s=4.0,
        export_imu_preroll_s=0.5,
    )

    imu_records = _read_jsonl(output / "imu.jsonl")
    image_records = _read_jsonl(output / "images.jsonl")
    timestamps = [record["timestamp_monotonic_ns"] for record in imu_records[:4]]
    assert timestamps == [1_500_000_000, 1_505_000_000, 1_510_000_000, 1_515_000_000]
    assert image_records[0]["timestamp_monotonic_ns"] == 3_000_000_000
    assert imu_records[0]["timestamp_source"] == "offline_reconstructed_200hz_by_sample_index"
    assert summary["export_window"]["output_counts"]["imu_samples"] == len(imu_records)
    assert summary["imu_time"]["exported"]["dt_ms_max"] == 5.0


def test_process_head_vio_rosfree_writes_head_pose_without_rosbag(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    output = tmp_path / "processed" / "openvins_c0"
    runner = tmp_path / "fake_run_csv_msckf.py"
    runner.write_text(
        "#!/usr/bin/env python3\n"
        "import sys\n"
        "out = sys.argv[sys.argv.index('--output') + 1]\n"
        "open(out, 'w').write('timestamp,p_x,p_y,p_z,q_x,q_y,q_z,q_w,v_x,v_y,v_z\\n1.0,1.0,2.0,3.0,0.0,0.0,0.0,1.0,0,0,0\\n')\n",
        encoding="utf-8",
    )
    runner.chmod(0o755)

    summary = process_head_vio_rosfree(
        session_dir=session,
        output_dir=output,
        cameras_path=Path("configs/cameras.yaml"),
        camera_ids=["C0"],
        runner_path=runner,
    )

    head_pose = _read_jsonl(output / "head_pose.jsonl")[0]
    assert summary["ok"] is True
    assert summary["steps"]["rosfree_run"]["ok"] is True
    assert summary["steps"]["rosfree_inputs"]["camera_inputs"]["cam0"]["frames"] == 4
    assert summary["steps"]["rosfree_inputs"]["filters"]["imu_preroll_samples"] > 0
    assert summary["steps"]["rosfree_inputs"]["counts"]["imu_samples"] > 1000
    assert head_pose["timestamp_monotonic_ns"] == 1_000_000_000
    assert head_pose["timestamp_source"] == "openvins_rosfree_csv"
    assert head_pose["T_W_H"]["position"] == [1.0, 2.0, 3.0]
    assert not (output / "rosbag2").exists()


def test_process_head_vio_rosfree_passes_four_camera_inputs(tmp_path: Path) -> None:
    session = _make_p3_session(tmp_path)
    cameras = session / "cameras"
    c0_records = _read_jsonl(cameras / "frames.jsonl")
    quad_records = []
    for camera_id in ["C0", "C1", "C2", "C3"]:
        (cameras / camera_id).mkdir(parents=True, exist_ok=True)
        for record in c0_records:
            filename = Path(record["image_path"]).name
            (cameras / camera_id / filename).write_bytes(b"fake image bytes")
            copied = dict(record)
            copied["camera_id"] = camera_id
            copied["image_path"] = f"{camera_id}/{filename}"
            quad_records.append(copied)
    _write_jsonl(cameras / "frames.jsonl", quad_records)
    output = tmp_path / "processed" / "openvins_quad"
    runner = tmp_path / "fake_run_csv_msckf.py"
    argv_path = tmp_path / "runner_argv.json"
    runner.write_text(
        "#!/usr/bin/env python3\n"
        "import json, sys\n"
        f"open({str(argv_path)!r}, 'w').write(json.dumps(sys.argv))\n"
        "out = sys.argv[sys.argv.index('--output') + 1]\n"
        "open(out, 'w').write('timestamp,p_x,p_y,p_z,q_x,q_y,q_z,q_w,v_x,v_y,v_z\\n1.0,0,0,0,0,0,0,1,0,0,0\\n')\n",
        encoding="utf-8",
    )
    runner.chmod(0o755)

    summary = process_head_vio_rosfree(
        session_dir=session,
        output_dir=output,
        cameras_path=Path("configs/cameras.yaml"),
        camera_ids=["C1", "C2", "C0", "C3"],
        runner_path=runner,
        fail_on_not_ready=False,
    )
    argv = json.loads(argv_path.read_text(encoding="utf-8"))

    assert summary["ok"] is True
    assert "--stereo" in argv
    assert "--cam2-csv" in argv
    assert "--cam3-csv" in argv
    assert summary["steps"]["rosfree_inputs"]["camera_inputs"]["cam3"]["camera_id"] == "C3"


def test_filter_rosbag_records_limits_duration_and_strides_images() -> None:
    image_records = [{"timestamp_unix_ns": index * 1_000_000_000} for index in range(6)]
    imu_records = [{"timestamp_unix_ns": index * 500_000_000} for index in range(12)]

    images, imus, summary = _filter_rosbag_records(
        image_records,
        imu_records,
        max_duration_s=2.0,
        start_offset_s=1.0,
        image_stride=2,
    )

    assert [record["timestamp_unix_ns"] for record in images] == [1_000_000_000, 3_000_000_000]
    assert imus[0]["timestamp_unix_ns"] == 1_000_000_000
    assert imus[-1]["timestamp_unix_ns"] == 3_000_000_000
    assert summary["window_counts"] == {"images": 3, "imu_samples": 5}
    assert summary["output_counts"] == {"images": 2, "imu_samples": 5}


def test_fuse_motion_session_composes_head_and_wrist_pose(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=True, with_head_imu=True)
    imus = session / "imus"
    _write_jsonl(
        imus / "wrist_imu.jsonl",
        [
            {
                "sensor_id": "wrist_imu",
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "accel_mps2": [0.0, 0.0, 9.81],
                "gyro_radps": [0.1, 0.2, 0.3],
            }
        ],
    )
    output = tmp_path / "processed" / session.name
    wrist_visual = output / "wrist_visual"
    wrist_visual.mkdir(parents=True)
    _write_jsonl(
        output / "head_pose.jsonl",
        [
            {
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "T_W_H": _pose_dict([1.0, 0.0, 0.0]),
            }
        ],
    )
    _write_jsonl(
        wrist_visual / "wrist_visual_pose.jsonl",
        [
            {
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "T_H_B": _pose_dict([0.0, 2.0, 0.0]),
            }
        ],
    )

    summary = fuse_motion_session(session_dir=session, output_root=output)
    motion_record = _read_jsonl(output / "motion" / "motion_frame.jsonl")[0]
    wrist_record = _read_jsonl(output / "motion" / "wrist_fused_pose.jsonl")[0]

    assert summary["counts"]["motion_frames"] == 1
    assert motion_record["head"]["position"] == [1.0, 0.0, 0.0]
    assert motion_record["wrist"]["position"] == [1.0, 2.0, 0.0]
    assert motion_record["wrist"]["angular_velocity"] == [0.1, 0.2, 0.3]
    assert wrist_record["alignment"]["wrist_imu_ok"] is True


def test_fuse_motion_session_uses_wrist_gyro_for_orientation_continuity(tmp_path: Path) -> None:
    session = _make_session(tmp_path, with_frame=True, with_head_imu=True)
    imus = session / "imus"
    _write_jsonl(
        imus / "wrist_imu.jsonl",
        [
            {
                "sensor_id": "wrist_imu",
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "accel_mps2": [0.0, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, math.pi / 2.0],
            },
            {
                "sensor_id": "wrist_imu",
                "timestamp_unix_ns": 1_500_000_000,
                "timestamp_monotonic_ns": 1_500_000_000,
                "accel_mps2": [0.0, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, math.pi / 2.0],
            },
        ],
    )
    output = tmp_path / "processed" / session.name
    wrist_visual = output / "wrist_visual"
    wrist_visual.mkdir(parents=True)
    _write_jsonl(
        output / "head_pose.jsonl",
        [
            {
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "T_W_H": _pose_dict([0.0, 0.0, 0.0]),
            },
            {
                "timestamp_unix_ns": 2_000_000_000,
                "timestamp_monotonic_ns": 2_000_000_000,
                "T_W_H": _pose_dict([0.0, 0.0, 0.0]),
            },
        ],
    )
    _write_jsonl(
        wrist_visual / "wrist_visual_pose.jsonl",
        [
            {
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "T_H_B": _pose_dict([0.0, 0.0, 0.0]),
            },
            {
                "timestamp_unix_ns": 2_000_000_000,
                "timestamp_monotonic_ns": 2_000_000_000,
                "T_H_B": _pose_dict([0.0, 0.0, 0.0]),
            },
        ],
    )

    summary = fuse_motion_session(
        session_dir=session,
        output_root=output,
        visual_correction_alpha=0.35,
    )
    wrist_records = _read_jsonl(output / "motion" / "wrist_fused_pose.jsonl")
    second_orientation = wrist_records[1]["T_W_B"]["orientation_wxyz"]

    assert summary["wrist_imu_fusion"]["imu_propagated_frames"] == 1
    assert wrist_records[1]["fusion"]["uses_wrist_gyro"] is True
    assert abs(second_orientation[3]) > 0.1


def test_prepare_euroc_openvins_session_exports_stereo_layout(tmp_path: Path) -> None:
    mav0 = _make_euroc_mav0(tmp_path)
    config_source = _make_euroc_config_source(tmp_path)
    output = tmp_path / "processed" / "euroc"

    summary = prepare_euroc_openvins_session(
        mav0_dir=mav0,
        output_dir=output,
        config_source_dir=config_source,
    )

    assert summary["counts"] == {"cam0_images": 2, "cam1_images": 2, "imu_samples": 2}
    image_records = _read_jsonl(output / "images.jsonl")
    imu_records = _read_jsonl(output / "imu.jsonl")
    assert {record["topic"] for record in image_records} == {"/cam0/image_raw", "/cam1/image_raw"}
    assert imu_records[0]["topic"] == "/imu0"
    assert (output / "config" / "estimator_config.yaml").exists()
    assert (output / "euroc_openvins_session_manifest.json").exists()


def test_dashboard_imu_scan_helper_formats_devices(monkeypatch) -> None:
    async def fake_scan(timeout_s: float):
        assert timeout_s == 1.5
        return [("WT901", "AA:BB:CC:DD:EE:FF")]

    monkeypatch.setattr(dashboard_server, "scan_wt_devices", fake_scan)

    assert dashboard_server._scan_imu_devices(1.5) == [
        {
            "name": "USB adapter ACM0",
            "address": "",
            "transport": "serial_adapter",
            "adapter_port": "/dev/ttyACM0",
            "adapter_passive": True,
        },
        {
            "name": "USB adapter ACM1",
            "address": "",
            "transport": "serial_adapter",
            "adapter_port": "/dev/ttyACM1",
            "adapter_passive": True,
        },
        {"name": "WT901", "address": "AA:BB:CC:DD:EE:FF"},
    ]


def test_wt901_ble_client_uses_ble_device_and_notify_only(monkeypatch) -> None:
    fake_bleak = _install_fake_bleak(
        monkeypatch,
        services=[
            _FakeService(
                wt901.WT901BleClient.SERVICE_UUID,
                [_FakeCharacteristic(wt901.WT901BleClient.READ_UUID, ["notify"])],
            )
        ],
    )
    samples = []
    connected = []
    client = wt901.WT901BleClient(
        "C4:65:91:2C:E2:20",
        "head_imu",
        samples.append,
        on_connected=lambda: connected.append(True),
    )

    asyncio.run(client.run(duration_s=0.0))

    assert fake_bleak.scanner_requested == [("C4:65:91:2C:E2:20", 20.0)]
    assert fake_bleak.client_targets == [fake_bleak.device]
    assert fake_bleak.client_instances[0].started_notify == [wt901.WT901BleClient.READ_UUID]
    assert fake_bleak.client_instances[0].writes == []
    assert connected == [True]
    assert len(samples) == 1
    assert samples[0].sensor_id == "head_imu"
    assert samples[0].quat_wxyz == [1.0, 0.0, 0.0, 0.0]
    assert samples[0].mag is None


def test_wt901_ble_client_reports_missing_notify_characteristic(monkeypatch) -> None:
    _install_fake_bleak(
        monkeypatch,
        services=[
            _FakeService(
                wt901.WT901BleClient.SERVICE_UUID,
                [_FakeCharacteristic("0000ffff-0000-1000-8000-00805f9a34fb", ["notify"])],
            )
        ],
    )
    client = wt901.WT901BleClient("C4:65:91:2C:E2:20", "head_imu", lambda sample: None)

    try:
        asyncio.run(client.run(duration_s=0.0))
    except RuntimeError as exc:
        message = str(exc)
    else:
        raise AssertionError("Expected missing notify characteristic to fail")

    assert wt901.WT901BleClient.READ_UUID in message
    assert "0000ffff-0000-1000-8000-00805f9a34fb" in message


def test_wt901_ble_client_aux_poll_writes_only_when_requested(monkeypatch) -> None:
    fake_bleak = _install_fake_bleak(
        monkeypatch,
        services=[
            _FakeService(
                wt901.WT901BleClient.SERVICE_UUID,
                [
                    _FakeCharacteristic(wt901.WT901BleClient.READ_UUID, ["notify"]),
                    _FakeCharacteristic(wt901.WT901BleClient.WRITE_UUID, ["write"]),
                ],
            )
        ],
    )
    client = wt901.WT901BleClient(
        "C4:65:91:2C:E2:20",
        "head_imu",
        lambda sample: None,
        aux_poll=True,
        aux_poll_start_delay_s=0.0,
    )

    asyncio.run(client.run(duration_s=0.25))

    writes = fake_bleak.client_instances[0].writes
    assert writes
    assert writes[0] == (wt901.WT901BleClient.WRITE_UUID, bytes([0xFF, 0xAA, 0x27, 0x3A, 0x00]))


def test_wt901_serial_adapter_scan_parser_matches_official_format() -> None:
    devices = wt901._parse_serial_adapter_scan(
        'WIT-LIST-#  1 :"WT901BLE67" 0xC465912CE220 -52\r\n'
        'WIT-LIST-#  2 :"WT901BLE67" 0xC564B94466D6 -61\r\n'
    )

    assert [(device.index, device.name, device.address, device.rssi) for device in devices] == [
        (1, "WT901BLE67", "C4:65:91:2C:E2:20", -52),
        (2, "WT901BLE67", "C5:64:B9:44:66:D6", -61),
    ]
    assert wt901._find_serial_adapter_index(devices, "c5:64:b9:44:66:d6") == 2


def test_wt901_serial_adapter_polls_mag_and_quaternion(monkeypatch) -> None:
    fake_port = _FakeSerialPort()
    monkeypatch.setattr(wt901, "_open_serial_port", lambda port, baudrate, timeout_s: fake_port)

    client = wt901.WT901SerialAdapterClient(
        "/dev/ttyACM0",
        "head_imu",
        lambda sample: None,
        device_index=2,
        aux_poll=True,
        aux_poll_start_delay_s=0.0,
    )

    client.run(duration_s=0.25)

    assert b"AT+CONNECT=2\r\n" in fake_port.writes
    assert bytes([0xFF, 0xAA, 0x27, 0x3A, 0x00]) in fake_port.writes
    assert bytes([0xFF, 0xAA, 0x27, 0x51, 0x00]) in fake_port.writes
    assert fake_port.closed is True


def test_wt901_packet_parser_attaches_quaternion_to_samples() -> None:
    samples = []
    parser = wt901.WT901PacketParser("wrist_imu", samples.append)
    quat_packet = bytearray([0x55, 0x71, 0x51] + [0x00] * 17)
    quat_packet[4:12] = bytes([0x00, 0x40, 0x00, 0x20, 0x00, 0x10, 0x00, 0x08])
    imu_packet = bytearray([0x55, 0x61] + [0x00] * 18)
    imu_packet[6:8] = bytes([0x00, 0x20])

    parser.feed(bytes(quat_packet))
    parser.feed(bytes(imu_packet))

    assert len(samples) == 1
    assert samples[0].quat_wxyz == [0.5, 0.25, 0.125, 0.0625]


def test_wt901_packet_parser_reconstructs_batched_sample_times() -> None:
    samples = []
    parser = wt901.WT901PacketParser(
        "head_imu",
        samples.append,
        timestamp_mode="reconstructed-rate",
        sample_rate_hz=200.0,
    )
    imu_packet = bytes(bytearray([0x55, 0x61] + [0x00] * 18))

    parser.feed(
        imu_packet * 3,
        receive_unix_ns=1_000_000_010_000_000,
        receive_monotonic_ns=10_000_000_000,
    )

    assert len(samples) == 3
    assert [sample.timestamp_monotonic_ns for sample in samples] == [
        9_990_000_000,
        9_995_000_000,
        10_000_000_000,
    ]
    assert all(sample.timestamp_source == "reconstructed_200hz_from_host_receive" for sample in samples)
    assert samples[0].host_receive_monotonic_ns == 10_000_000_000
    assert samples[0].timestamp_reconstruction["batch_size"] == 3


def test_wt901_serial_adapter_connects_before_stopping_scan(monkeypatch) -> None:
    fake_port = _FakeSerialPort(
        reads=[
            b'WIT-LIST-#  2 :"WT901BLE67" 0xC465912CE220 -52\r\n',
            b"",
        ]
    )
    monkeypatch.setattr(wt901, "_open_serial_port", lambda port, baudrate, timeout_s: fake_port)

    client = wt901.WT901SerialAdapterClient(
        "/dev/ttyACM0",
        "head_imu",
        lambda sample: None,
        address="C4:65:91:2C:E2:20",
    )

    client.run(duration_s=0.01)

    scan_start = fake_port.writes.index(b"AT+SCAN=1\r\n")
    connect = fake_port.writes.index(b"AT+CONNECT=2\r\n")
    scan_stop = fake_port.writes.index(b"AT+SCAN=0\r\n")
    assert scan_start < connect < scan_stop


def test_wt901_serial_adapter_passive_read_does_not_send_connect(monkeypatch) -> None:
    fake_port = _FakeSerialPort(reads=[bytes(bytearray([0x55, 0x61] + [0x00] * 18))])
    monkeypatch.setattr(wt901, "_open_serial_port", lambda port, baudrate, timeout_s: fake_port)
    samples = []

    client = wt901.WT901SerialAdapterClient(
        "/dev/ttyACM1",
        "head_imu",
        samples.append,
        passive=True,
    )

    client.run(duration_s=0.01)

    assert samples
    assert b"AT+CONNECT=1\r\n" not in fake_port.writes
    assert b"AT+SCAN=1\r\n" not in fake_port.writes
    assert b"AT+SCAN=0\r\n" not in fake_port.writes
    assert fake_port.closed is True


def _make_session(tmp_path: Path, *, with_frame: bool, with_head_imu: bool) -> Path:
    session = tmp_path / "session_20260612_010000"
    cameras = session / "cameras"
    imus = session / "imus"
    (cameras / "C0").mkdir(parents=True)
    imus.mkdir(parents=True)
    (session / "session_manifest.json").write_text("{}\n", encoding="utf-8")
    if with_frame:
        (cameras / "C0" / "00000000.jpg").write_bytes(b"fake image bytes")
        _write_jsonl(
            cameras / "frames.jsonl",
            [
                {
                    "group_id": 0,
                    "camera_id": "C0",
                    "timestamp_unix_ns": 1000,
                    "timestamp_monotonic_ns": 2000,
                    "timestamp_source": "host_retrieve",
                    "skew_us": 0.0,
                    "image_path": "C0/00000000.jpg",
                    "width": 640,
                    "height": 480,
                }
            ],
        )
    if with_head_imu:
        _write_jsonl(
            imus / "head_imu.jsonl",
            [
                {
                    "sensor_id": "head_imu",
                    "timestamp_unix_ns": 900,
                    "timestamp_monotonic_ns": 1900,
                    "timestamp_source": "host_receive",
                    "accel_mps2": [0.0, 0.0, 9.8],
                    "gyro_radps": [0.0, 0.0, 0.0],
                    "euler_deg": [0.0, 0.0, 0.0],
                }
            ],
        )
    return session


def _make_p3_session(tmp_path: Path) -> Path:
    session = tmp_path / "session_20260612_020000"
    cameras = session / "cameras"
    imus = session / "imus"
    (cameras / "C0").mkdir(parents=True)
    imus.mkdir(parents=True)
    (session / "session_manifest.json").write_text("{}\n", encoding="utf-8")

    frame_records = []
    for index, timestamp in enumerate([1_000_000_000, 3_000_000_000, 5_000_000_000, 7_000_000_000]):
        filename = f"{index:08d}.jpg"
        (cameras / "C0" / filename).write_bytes(b"fake image bytes")
        frame_records.append(
            {
                "group_id": index,
                "camera_id": "C0",
                "timestamp_unix_ns": timestamp,
                "timestamp_monotonic_ns": timestamp,
                "timestamp_source": "host_retrieve",
                "skew_us": 0.0,
                "image_path": f"C0/{filename}",
                "width": 640,
                "height": 480,
            }
        )
    _write_jsonl(cameras / "frames.jsonl", frame_records)

    imu_records = []
    for index in range(1500):
        timestamp = 900_000_000 + index * 5_000_000
        accel_x = 0.0 if index < 20 else (1.2 if index % 2 == 0 else -1.2)
        imu_records.append(
            {
                "sensor_id": "head_imu",
                "timestamp_unix_ns": timestamp,
                "timestamp_monotonic_ns": timestamp,
                "timestamp_source": "host_receive",
                "accel_mps2": [accel_x, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, 0.05 if index >= 20 else 0.0],
                "euler_deg": [0.0, 0.0, 0.0],
            }
        )
    _write_jsonl(imus / "head_imu.jsonl", imu_records)
    return session


def _startup_gap_then_clean_imu_records() -> list[dict]:
    records = []
    for index in range(10):
        timestamp = index * 60_000_000
        records.append(
            {
                "sensor_id": "head_imu",
                "timestamp_unix_ns": timestamp,
                "timestamp_monotonic_ns": timestamp,
                "timestamp_source": "host_receive",
                "accel_mps2": [0.0, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, 0.0],
            }
        )
    for index in range(1500):
        timestamp = 900_000_000 + index * 5_000_000
        accel_x = 0.0 if index < 20 else (1.2 if index % 2 == 0 else -1.2)
        records.append(
            {
                "sensor_id": "head_imu",
                "timestamp_unix_ns": timestamp,
                "timestamp_monotonic_ns": timestamp,
                "timestamp_source": "host_receive",
                "accel_mps2": [accel_x, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, 0.05 if index >= 20 else 0.0],
            }
        )
    return records


def _make_quad_session(tmp_path: Path) -> Path:
    session = tmp_path / "session_20260612_030000"
    cameras = session / "cameras"
    imus = session / "imus"
    imus.mkdir(parents=True)
    (session / "session_manifest.json").write_text("{}\n", encoding="utf-8")
    records = []
    for camera_id in ["C0", "C1", "C2", "C3"]:
        (cameras / camera_id).mkdir(parents=True)
        filename = "00000000.jpg"
        (cameras / camera_id / filename).write_bytes(b"fake image")
        records.append(
            {
                "group_id": 0,
                "camera_id": camera_id,
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "timestamp_source": "host_retrieve",
                "skew_us": 0.0,
                "image_path": f"{camera_id}/{filename}",
                "width": 1600,
                "height": 1200,
            }
        )
    _write_jsonl(cameras / "frames.jsonl", records)
    _write_jsonl(
        imus / "head_imu.jsonl",
        [
            {
                "sensor_id": "head_imu",
                "timestamp_unix_ns": 1_000_000_000,
                "timestamp_monotonic_ns": 1_000_000_000,
                "accel_mps2": [0.0, 0.0, 9.81],
                "gyro_radps": [0.0, 0.0, 0.0],
            }
        ],
    )
    return session


def _make_euroc_mav0(tmp_path: Path) -> Path:
    mav0 = tmp_path / "mav0"
    for camera_id in ["cam0", "cam1"]:
        data_dir = mav0 / camera_id / "data"
        data_dir.mkdir(parents=True)
        for timestamp in [1000, 2000]:
            (data_dir / f"{timestamp}.png").write_bytes(b"fake image")
        (mav0 / camera_id / "data.csv").write_text(
            "#timestamp [ns],filename\n1000,1000.png\n2000,2000.png\n",
            encoding="utf-8",
        )
    (mav0 / "imu0").mkdir(parents=True)
    (mav0 / "imu0" / "data.csv").write_text(
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
        "900,0.1,0.2,0.3,1.0,2.0,9.8\n"
        "1900,0.2,0.3,0.4,1.1,2.1,9.7\n",
        encoding="utf-8",
    )
    return mav0


def _make_euroc_config_source(tmp_path: Path) -> Path:
    config = tmp_path / "config_source"
    config.mkdir()
    for name in ["estimator_config.yaml", "kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"]:
        (config / name).write_text("%YAML:1.0\n", encoding="utf-8")
    return config


def _write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")


def _read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _pose_dict(position: list[float]) -> dict:
    return {
        "position": position,
        "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "matrix": [
            [1.0, 0.0, 0.0, position[0]],
            [0.0, 1.0, 0.0, position[1]],
            [0.0, 0.0, 1.0, position[2]],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }


class _FakeCharacteristic:
    def __init__(self, uuid: str, properties: list[str]) -> None:
        self.uuid = uuid
        self.properties = properties


class _FakeService:
    def __init__(self, uuid: str, characteristics: list[_FakeCharacteristic]) -> None:
        self.uuid = uuid
        self.characteristics = characteristics


class _FakeDevice:
    name = "WT901BLE"
    address = "C4:65:91:2C:E2:20"


class _FakeSerialPort:
    def __init__(self, reads: list[bytes] | None = None) -> None:
        self.writes = []
        self.closed = False
        self.reads = list(reads or [])

    def write(self, data) -> None:
        self.writes.append(bytes(data))

    def flush(self) -> None:
        return

    def read(self, size: int) -> bytes:
        if self.reads:
            return self.reads.pop(0)
        return b""

    def close(self) -> None:
        self.closed = True


def _install_fake_bleak(monkeypatch, *, services: list[_FakeService]):
    fake_bleak = types.SimpleNamespace()
    fake_bleak.device = _FakeDevice()
    fake_bleak.scanner_requested = []
    fake_bleak.client_targets = []
    fake_bleak.client_instances = []

    class FakeScanner:
        @staticmethod
        async def find_device_by_address(address, timeout):
            fake_bleak.scanner_requested.append((address, timeout))
            return fake_bleak.device

        @staticmethod
        async def discover(timeout):
            return [fake_bleak.device]

    class FakeClient:
        def __init__(self, target, timeout, **kwargs):
            self.target = target
            self.timeout = timeout
            self.kwargs = kwargs
            self.services = services
            self.started_notify = []
            self.stopped_notify = []
            self.writes = []
            fake_bleak.client_targets.append(target)
            fake_bleak.client_instances.append(self)

        async def __aenter__(self):
            return self

        async def __aexit__(self, exc_type, exc, tb):
            return False

        async def start_notify(self, uuid, callback):
            self.started_notify.append(uuid)
            callback(uuid, bytes([0x55, 0x61] + [0x00] * 18))

        async def stop_notify(self, uuid):
            self.stopped_notify.append(uuid)

        async def write_gatt_char(self, uuid, data):
            self.writes.append((uuid, bytes(data)))

    fake_bleak.BleakScanner = FakeScanner
    fake_bleak.BleakClient = FakeClient
    monkeypatch.setitem(sys.modules, "bleak", fake_bleak)
    return fake_bleak
