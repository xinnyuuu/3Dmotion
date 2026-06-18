from __future__ import annotations

import json
from pathlib import Path

from packages.head_vio_bridge.p3_head_vio import check_head_vio_readiness, prepare_p3_head_vio
from packages.head_vio_bridge.process_head_vio_session import process_head_vio_session
from packages.head_vio_bridge.euroc_session import prepare_euroc_openvins_session
from packages.head_vio_bridge.openvins_session import prepare_openvins_session
from packages.head_vio_bridge.openvins_session import _filter_rosbag_records
from packages.capture_dashboard import server as dashboard_server
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

    assert dashboard_server._scan_imu_devices(1.5) == [{"name": "WT901", "address": "AA:BB:CC:DD:EE:FF"}]


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
    for index in range(45):
        timestamp = 900_000_000 + index * 160_000_000
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
