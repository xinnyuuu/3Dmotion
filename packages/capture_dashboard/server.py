from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import os
import re
import signal
import socketserver
import subprocess
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from packages.imu_ble_bridge.wt901 import (
    WT901BleClient,
    WT901SerialAdapterClient,
    scan_serial_adapter_devices,
    scan_wt_devices,
    write_jsonl_sample,
)
from packages.quad_camera_capture.capture import CameraSource, QuadCameraCapture
from packages.quad_camera_capture.v4l2 import find_capture_devices, get_camera_config
from packages.session_tools.validate_session import validate_session


DEFAULT_CAMERA_WIDTH = 1600
DEFAULT_CAMERA_HEIGHT = 1200
DEFAULT_CAMERA_FPS = 25.0
DEFAULT_CAMERA_FORMAT = "MJPG"
DEFAULT_PREVIEW_FPS = 8.0
DEFAULT_IMU_SCAN_TIMEOUT_S = 20.0
IMU_NO_DATA_WARN_S = 5.0
IMU_STALE_SAMPLE_WARN_S = 3.0
DEFAULT_HEAD_IMU_ADAPTER_PORT = "/dev/ttyACM1"
DEFAULT_WRIST_IMU_ADAPTER_PORT = "/dev/ttyACM0"
KNOWN_IMU_DEVICES = [
    {
        "name": "USB adapter ACM0",
        "address": "",
        "transport": "serial_adapter",
        "adapter_port": DEFAULT_WRIST_IMU_ADAPTER_PORT,
        "adapter_passive": True,
    },
    {
        "name": "USB adapter ACM1",
        "address": "",
        "transport": "serial_adapter",
        "adapter_port": DEFAULT_HEAD_IMU_ADAPTER_PORT,
        "adapter_passive": True,
    },
]


@dataclass
class PreviewState:
    key: str
    source: object
    fourcc: str
    width: int
    height: int
    fps: float
    lock: threading.Lock
    stop_event: threading.Event
    thread: threading.Thread | None = None
    cap: object | None = None
    last_jpeg: bytes | None = None
    last_ok_at: float = 0.0
    failed_reads: int = 0
    status: str = "opening"
    stop_incomplete: bool = False


class PreviewCaptureManager:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.captures: dict[str, PreviewState] = {}

    def get(self, source: object, fourcc: str, width: int, height: int, fps: float) -> PreviewState:
        fourcc = _normalize_fourcc(fourcc)
        with self.lock:
            state = self.captures.get(source)
            needs_open = (
                state is None
                or state.fourcc != fourcc
                or state.width != width
                or state.height != height
                or state.fps != fps
            )
            if needs_open:
                if state is not None:
                    self._stop_state(state)
                    time.sleep(0.12)
                state = self._start_state(source, fourcc, width, height, fps)
                self.captures[source] = state
            return state

    def read_jpeg(self, source: object, fourcc: str, width: int, height: int, fps: float) -> bytes | None:
        state = self.get(source, fourcc, width, height, fps)
        with state.lock:
            return state.last_jpeg

    def stop_all(self) -> None:
        with self.lock:
            states = list(self.captures.values())
            self.captures.clear()
        for state in states:
            self._stop_state(state)

    def snapshot(self) -> dict:
        with self.lock:
            states = list(self.captures.values())
        now = time.monotonic()
        result = {}
        for state in states:
            with state.lock:
                result[state.source] = {
                    "key": state.key,
                    "fourcc": state.fourcc,
                    "width": state.width,
                    "height": state.height,
                    "fps": state.fps,
                    "status": state.status,
                    "has_jpeg": state.last_jpeg is not None,
                    "last_ok_age_s": round(now - state.last_ok_at, 2) if state.last_ok_at else None,
                    "failed_reads": state.failed_reads,
                    "thread_alive": bool(state.thread and state.thread.is_alive()),
                    "stop_incomplete": state.stop_incomplete,
                }
        return result

    def _start_state(self, source: object, fourcc: str, width: int, height: int, fps: float) -> PreviewState:
        state = PreviewState(
            key=_stream_key(source, fourcc, width, height, fps),
            source=source,
            fourcc=fourcc,
            width=width,
            height=height,
            fps=fps,
            lock=threading.Lock(),
            stop_event=threading.Event(),
        )
        state.thread = threading.Thread(target=self._reader_loop, args=(state,), daemon=True)
        state.thread.start()
        return state

    def _stop_state(self, state: PreviewState) -> None:
        state.stop_event.set()
        if state.thread is not None and state.thread is not threading.current_thread():
            state.thread.join(timeout=1.5)
        still_alive = bool(state.thread and state.thread.is_alive())
        if still_alive and state.cap is not None:
            with contextlib.suppress(Exception):
                state.cap.release()
        with state.lock:
            state.cap = None
            state.last_jpeg = None
            state.status = "stopped"
            state.stop_incomplete = still_alive

    def _open_capture(self, state: PreviewState):
        import cv2

        if not _is_valid_capture_source(state.source):
            with state.lock:
                state.status = f"invalid source: {state.source}"
            return None
        cap = cv2.VideoCapture(state.source, cv2.CAP_V4L2)
        if cap.isOpened():
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 700)
            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 700)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*state.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, state.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, state.height)
            cap.set(cv2.CAP_PROP_FPS, state.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _reader_loop(self, state: PreviewState) -> None:
        import cv2

        delay = 1.0 / max(1.0, state.fps)
        while not state.stop_event.is_set():
            cap = self._open_capture(state)
            with state.lock:
                state.cap = cap
                state.status = "streaming" if cap is not None and cap.isOpened() else state.status if state.status.startswith("invalid source") else "open failed"
                state.failed_reads = 0
            if cap is None:
                state.stop_event.wait(0.35)
                continue
            if not cap.isOpened():
                cap.release()
                state.stop_event.wait(0.35)
                continue

            while not state.stop_event.is_set():
                started_at = time.monotonic()
                ok, frame = cap.read()
                now = time.monotonic()
                if ok and frame is not None:
                    encode_ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    with state.lock:
                        state.last_ok_at = now
                        state.failed_reads = 0
                        state.status = "streaming"
                        if encode_ok:
                            state.last_jpeg = encoded.tobytes()
                else:
                    with state.lock:
                        state.failed_reads += 1
                        state.status = "read timeout"
                        should_reopen = state.failed_reads >= 3
                    if should_reopen:
                        break

                remaining = delay - (time.monotonic() - started_at)
                if remaining > 0:
                    state.stop_event.wait(remaining)

            cap.release()
            with state.lock:
                if state.cap is cap:
                    state.cap = None
                if not state.stop_event.is_set():
                    state.status = "reopening"
            if not state.stop_event.is_set():
                state.stop_event.wait(0.15)


@dataclass
class JobStatus:
    kind: str
    active: bool = False
    started_at: float | None = None
    finished_at: float | None = None
    message: str = "idle"
    error: str | None = None
    output: str | None = None

    def as_dict(self) -> dict:
        data = asdict(self)
        now = time.monotonic()
        data["elapsed_s"] = round(now - self.started_at, 2) if self.active and self.started_at else 0.0
        return data


@dataclass
class ImuSlotStatus:
    slot: str
    selected_name: str | None = None
    selected_address: str | None = None
    transport: str = "ble"
    adapter_port: str | None = None
    adapter_device_index: int | None = None
    adapter_passive: bool = False
    connection_state: str = "idle"
    connection_monotonic_ns: int | None = None
    last_sample_monotonic_ns: int | None = None
    last_error: str | None = None
    recording: bool = False
    output: str | None = None
    last_sample: dict | None = None
    sample_count: int = 0

    def as_dict(self) -> dict:
        data = asdict(self)
        if self.last_sample_monotonic_ns is not None:
            age_s = (time.monotonic_ns() - self.last_sample_monotonic_ns) / 1_000_000_000
            data["last_sample_age_s"] = round(age_s, 2)
        else:
            data["last_sample_age_s"] = None
        if self.connection_monotonic_ns is not None:
            age_s = (time.monotonic_ns() - self.connection_monotonic_ns) / 1_000_000_000
            data["connection_age_s"] = round(age_s, 2)
        else:
            data["connection_age_s"] = None
        return data


class CaptureJobs:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.camera = JobStatus(kind="camera")
        self.recording = JobStatus(kind="record")
        self.visualization = JobStatus(kind="visualization")
        self.imu_slots = {
            "head_imu": ImuSlotStatus(slot="head_imu"),
            "wrist_imu": ImuSlotStatus(slot="wrist_imu"),
        }
        self._camera_stop = threading.Event()
        self._imu_stop_events = {
            "head_imu": threading.Event(),
            "wrist_imu": threading.Event(),
        }
        self._imu_record_paths: dict[str, Path | None] = {"head_imu": None, "wrist_imu": None}
        self._imu_live_paths: dict[str, Path | None] = {"head_imu": None, "wrist_imu": None}
        self._session_dir: Path | None = None
        self._camera_thread: threading.Thread | None = None
        self._imu_threads: dict[str, threading.Thread | None] = {"head_imu": None, "wrist_imu": None}
        self._visualization_processes: list[subprocess.Popen] = []
        self.session_summary: dict | None = None

    def start_record(self, payload: dict) -> JobStatus:
        with self.lock:
            if self.recording.active:
                raise RuntimeError("Recording is already running.")
            self._camera_stop = threading.Event()
            session_dir = _session_dir(payload)
            session_dir.mkdir(parents=True, exist_ok=True)
            self._session_dir = session_dir
            self._camera_thread = None
            self.session_summary = None
            self.recording = JobStatus(
                kind="record",
                active=True,
                started_at=time.monotonic(),
                message="recording",
                output=str(session_dir),
            )
            self._prepare_imu_recording_locked(session_dir)
            self.camera = JobStatus(kind="camera", active=False, message="not selected", output=str(session_dir / "cameras"))

        self._write_session_manifest(session_dir, payload)
        camera_sources = _camera_sources_from_payload(payload)
        if camera_sources:
            with self.lock:
                self.camera = JobStatus(kind="camera", active=True, started_at=time.monotonic(), message="starting", output=str(session_dir / "cameras"))
            thread = threading.Thread(target=self._run_camera, args=(camera_sources, payload, session_dir / "cameras", self._camera_stop), daemon=True)
            with self.lock:
                self._camera_thread = thread
            thread.start()
        return self.recording

    def stop_record(self) -> JobStatus:
        self._camera_stop.set()
        session_dir = None
        camera_thread = None
        with self.lock:
            for slot in self._imu_record_paths:
                self._imu_record_paths[slot] = None
                self.imu_slots[slot].recording = False
            if self.camera.active:
                self.camera.message = "stopping"
            if self.recording.active:
                self.recording.active = False
                self.recording.finished_at = time.monotonic()
                self.recording.message = "stopped; validating"
            session_dir = self._session_dir
            camera_thread = self._camera_thread
        if session_dir is not None:
            threading.Thread(target=self._finalize_session_summary, args=(session_dir, camera_thread), daemon=True).start()
        with self.lock:
            return self.recording

    def select_imu(self, payload: dict) -> ImuSlotStatus:
        slot = _imu_slot(payload)
        address = str(payload.get("address") or "").strip()
        name = str(payload.get("name") or "WT IMU").strip()
        transport = str(payload.get("transport") or "ble").strip()
        adapter_port = str(payload.get("adapter_port") or "").strip() or None
        adapter_device_index = _optional_int(payload.get("adapter_device_index"))
        adapter_passive = _optional_bool(payload.get("adapter_passive"))
        if transport not in {"ble", "serial_adapter"}:
            raise RuntimeError(f"Unknown IMU transport: {transport}")
        if transport == "serial_adapter" and not adapter_port:
            raise RuntimeError("Missing serial adapter port.")
        if not address and not (transport == "serial_adapter" and adapter_passive):
            raise RuntimeError("Missing IMU BLE address.")

        with self.lock:
            current = self.imu_slots[slot]
            same_target = (
                current.selected_address == address
                and current.transport == transport
                and current.adapter_port == adapter_port
                and current.adapter_device_index == adapter_device_index
                and current.adapter_passive == adapter_passive
            )
            if same_target and current.connection_state in {"connecting", "waiting_data", "connected"}:
                return current

        self._imu_stop_events[slot].set()
        with self.lock:
            self._imu_stop_events[slot] = threading.Event()
            self._imu_record_paths[slot] = None
            self.imu_slots[slot] = ImuSlotStatus(
                slot=slot,
                selected_name=name,
                selected_address=address,
                transport=transport,
                adapter_port=adapter_port,
                adapter_device_index=adapter_device_index,
                adapter_passive=adapter_passive,
                connection_state="connecting",
                connection_monotonic_ns=time.monotonic_ns(),
            )
            stop_event = self._imu_stop_events[slot]
        thread = threading.Thread(
            target=self._run_imu_connection,
            args=(slot, address, transport, adapter_port, adapter_device_index, adapter_passive, stop_event),
            daemon=True,
        )
        with self.lock:
            self._imu_threads[slot] = thread
        thread.start()
        return self.imu_slots[slot]

    def disconnect_imu(self, slot: str) -> ImuSlotStatus:
        if slot not in self.imu_slots:
            raise RuntimeError(f"Unknown IMU slot: {slot}")
        self._imu_stop_events[slot].set()
        with self.lock:
            self._imu_record_paths[slot] = None
            self.imu_slots[slot].recording = False
            self.imu_slots[slot].connection_state = "disconnecting"
            return self.imu_slots[slot]

    def snapshot(self) -> dict:
        with self.lock:
            self._refresh_visualization_locked()
            self._refresh_imu_health_locked()
            return {
                "recording": self.recording.as_dict(),
                "camera": self.camera.as_dict(),
                "visualization": self.visualization.as_dict(),
                "imus": {slot: status.as_dict() for slot, status in self.imu_slots.items()},
                "session_summary": self.session_summary,
            }

    def start_visualization(self, payload: dict) -> JobStatus:
        with self.lock:
            self._refresh_visualization_locked()
            if self.visualization.active:
                raise RuntimeError("Visualization is already running.")
            if self.recording.active:
                raise RuntimeError("Stop recording before starting live visualization.")

        camera_sources = _camera_sources_from_payload(payload)
        if not camera_sources:
            raise RuntimeError("Select at least one camera before starting visualization.")

        repo_root = Path(__file__).resolve().parents[2]
        log_dir = repo_root / "data" / "processed" / "live_visualization"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        live_log = log_dir / f"world_anchor_live_{stamp}.log"
        rviz_log = log_dir / f"rviz_live_{stamp}.log"
        head_imu_live = log_dir / f"head_imu_live_{stamp}.jsonl"
        wrist_imu_live = log_dir / f"wrist_imu_live_{stamp}.jsonl"
        head_imu_live.write_text("", encoding="utf-8")
        wrist_imu_live.write_text("", encoding="utf-8")
        cfg = _record_profile_from_payload(payload)
        live_fps = min(float(cfg["fps"]), 10.0)
        ros_domain_id = str(payload.get("ros_domain_id") or "73")
        source_args = " ".join(
            _shell_quote(item)
            for source in camera_sources
            for item in ("--source", f"{source.camera_id}:{source.source}")
        )
        live_cmd = (
            f"cd {_shell_quote(str(repo_root))} && "
            "source /opt/ros/humble/setup.bash && "
            "source .venv/bin/activate && "
            f"export ROS_DOMAIN_ID={_shell_quote(ros_domain_id)} ROS_LOCALHOST_ONLY=1 && "
            "python scripts/process_world_anchor_session.py live "
            f"{source_args} "
            "--world-tags configs/world_tags.yaml "
            "--bracelet configs/bracelet.yaml "
            f"--width {_shell_quote(str(cfg['width']))} "
            f"--height {_shell_quote(str(cfg['height']))} "
            f"--fps {_shell_quote(f'{live_fps:g}')} "
            f"--head-imu-live-jsonl {_shell_quote(str(head_imu_live))} "
            f"--wrist-imu-live-jsonl {_shell_quote(str(wrist_imu_live))} "
            "--ros-publish"
        )
        if payload.get("hands"):
            live_cmd += " --hands"
        rviz_cmd = (
            f"cd {_shell_quote(str(repo_root / 'ros2_ws'))} && "
            "unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            f"export ROS_DOMAIN_ID={_shell_quote(ros_domain_id)} ROS_LOCALHOST_ONLY=1 && "
            "ros2 launch vimas_motion_bringup motion_live_rviz.launch.py"
        )

        try:
            live_file = live_log.open("w", encoding="utf-8")
            rviz_file = rviz_log.open("w", encoding="utf-8")
            live_process = subprocess.Popen(
                ["bash", "-lc", live_cmd],
                stdout=live_file,
                stderr=subprocess.STDOUT,
                cwd=repo_root,
                start_new_session=True,
            )
            time.sleep(0.8)
            rviz_process = subprocess.Popen(
                ["bash", "-lc", rviz_cmd],
                stdout=rviz_file,
                stderr=subprocess.STDOUT,
                cwd=repo_root,
                start_new_session=True,
            )
        except Exception:
            with contextlib.suppress(Exception):
                live_file.close()
            with contextlib.suppress(Exception):
                rviz_file.close()
            raise

        with self.lock:
            self._imu_live_paths["head_imu"] = head_imu_live
            self._imu_live_paths["wrist_imu"] = wrist_imu_live
            self._visualization_processes = [live_process, rviz_process]
            self.visualization = JobStatus(
                kind="visualization",
                active=True,
                started_at=time.monotonic(),
                message="running",
                output=f"live={live_log} rviz={rviz_log} head_imu={head_imu_live} wrist_imu={wrist_imu_live}",
            )
            return self.visualization

    def stop_visualization(self) -> JobStatus:
        with self.lock:
            processes = list(self._visualization_processes)
            self._visualization_processes = []
            for slot in self._imu_live_paths:
                self._imu_live_paths[slot] = None
            self.visualization.message = "stopping"

        for process in processes:
            self._terminate_process_group(process)

        with self.lock:
            self.visualization.active = False
            self.visualization.finished_at = time.monotonic()
            self.visualization.message = "stopped"
            return self.visualization

    def shutdown(self, timeout_s: float = 3.0) -> None:
        self._camera_stop.set()
        for event in self._imu_stop_events.values():
            event.set()
        with self.lock:
            visualization_processes = list(self._visualization_processes)
            self._visualization_processes = []
            camera_thread = self._camera_thread
            imu_threads = [thread for thread in self._imu_threads.values() if thread is not None]
            for slot in self._imu_record_paths:
                self._imu_record_paths[slot] = None
                self._imu_live_paths[slot] = None
                self.imu_slots[slot].recording = False
                if self.imu_slots[slot].connection_state not in {"idle", "disconnected", "failed"}:
                    self.imu_slots[slot].connection_state = "disconnecting"
            self.recording.active = False
            self.camera.active = False
            self.visualization.active = False

        deadline = time.monotonic() + max(0.1, timeout_s)
        for process in visualization_processes:
            self._terminate_process_group(process)
        if camera_thread is not None and camera_thread is not threading.current_thread():
            camera_thread.join(timeout=max(0.0, deadline - time.monotonic()))
        for thread in imu_threads:
            if thread is threading.current_thread():
                continue
            thread.join(timeout=max(0.0, deadline - time.monotonic()))

    def _refresh_visualization_locked(self) -> None:
        if not self._visualization_processes:
            if self.visualization.active:
                for slot in self._imu_live_paths:
                    self._imu_live_paths[slot] = None
                self.visualization.active = False
                self.visualization.finished_at = time.monotonic()
                self.visualization.message = "stopped"
            return
        running = [process for process in self._visualization_processes if process.poll() is None]
        if running:
            exited = [process.returncode for process in self._visualization_processes if process.poll() is not None]
            self._visualization_processes = running
            self.visualization.active = True
            if exited and any(code not in (0, None) for code in exited):
                self.visualization.message = f"running; child exited {exited}"
                self.visualization.error = f"One visualization child exited with codes {exited}. Check logs: {self.visualization.output}"
            return
        codes = [process.returncode for process in self._visualization_processes]
        self._visualization_processes = []
        for slot in self._imu_live_paths:
            self._imu_live_paths[slot] = None
        self.visualization.active = False
        self.visualization.finished_at = time.monotonic()
        if any(code not in (0, None) for code in codes):
            self.visualization.message = f"exited {codes}"
            self.visualization.error = f"Visualization processes exited with codes {codes}."
        else:
            self.visualization.message = "stopped"

    def _terminate_process_group(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            with contextlib.suppress(subprocess.TimeoutExpired):
                process.wait(timeout=1.0)

    def _finalize_session_summary(self, session_dir: Path, camera_thread: threading.Thread | None) -> None:
        if camera_thread is not None and camera_thread.is_alive():
            camera_thread.join(timeout=5.0)
        try:
            summary = validate_session(session_dir)
            summary_path = session_dir / "session_summary.json"
            summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
            with self.lock:
                self.session_summary = summary
                self.recording.message = "stopped; validated"
        except Exception as exc:
            with self.lock:
                self.session_summary = {
                    "session_dir": str(session_dir),
                    "ok_for_camera_replay": False,
                    "error": f"{exc}\n{traceback.format_exc()}",
                }
                self.recording.message = "stopped; validation failed"

    def _prepare_imu_recording_locked(self, session_dir: Path) -> None:
        imu_dir = session_dir / "imus"
        imu_dir.mkdir(parents=True, exist_ok=True)
        for slot, status in self.imu_slots.items():
            if status.connection_state == "connected":
                path = imu_dir / f"{slot}.jsonl"
                self._imu_record_paths[slot] = path
                status.recording = True
                status.output = str(path)
            else:
                self._imu_record_paths[slot] = None
                status.recording = False
                status.output = None

    def _write_session_manifest(self, session_dir: Path, payload: dict) -> None:
        manifest = {
            "session_dir": str(session_dir),
            "started_unix_ns": time.time_ns(),
            "camera": {
                "sources": {camera_id: payload.get(camera_id) for camera_id in ("C0", "C1", "C2", "C3") if payload.get(camera_id)},
                "tile_configs": payload.get("camera_configs") or {},
                "width": _optional_int(payload.get("width")) or DEFAULT_CAMERA_WIDTH,
                "height": _optional_int(payload.get("height")) or DEFAULT_CAMERA_HEIGHT,
                "fps": _optional_float(payload.get("fps")) or DEFAULT_CAMERA_FPS,
            },
            "imus": {slot: status.as_dict() for slot, status in self.imu_slots.items()},
        }
        (session_dir / "session_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    def _run_camera(self, sources: list[CameraSource], payload: dict, output_dir: Path, stop_event: threading.Event) -> None:
        try:
            fps = _optional_float(payload.get("fps")) or DEFAULT_CAMERA_FPS
            width = _optional_int(payload.get("width")) or DEFAULT_CAMERA_WIDTH
            height = _optional_int(payload.get("height")) or DEFAULT_CAMERA_HEIGHT
            with self.lock:
                self.camera.message = "capturing"
                self.camera.output = str(output_dir)
            QuadCameraCapture(
                sources=sources,
                output_dir=output_dir,
                target_fps=fps,
                width=width,
                height=height,
            ).run(
                duration_s=None,
                stop_event=stop_event,
            )
            with self.lock:
                self.camera.active = False
                self.camera.finished_at = time.monotonic()
                manifest_path = output_dir / "frames.jsonl"
                self.camera.message = "finished" if manifest_path.exists() and manifest_path.stat().st_size > 0 else "no frames captured"
        except Exception as exc:
            with self.lock:
                self.camera.active = False
                self.camera.finished_at = time.monotonic()
                self.camera.message = "failed"
                self.camera.error = f"{exc}\n{traceback.format_exc()}"

    def _run_imu_connection(
        self,
        slot: str,
        address: str,
        transport: str,
        adapter_port: str | None,
        adapter_device_index: int | None,
        adapter_passive: bool,
        stop_event: threading.Event,
    ) -> None:
        try:
            sensor_id = slot

            async def run_client() -> None:
                client = WT901BleClient(
                    address,
                    sensor_id,
                    lambda sample: self._handle_imu_sample(slot, sample, address, transport, adapter_port, adapter_device_index, adapter_passive),
                    on_connected=lambda: self._mark_imu_connected(slot, address, transport, adapter_port, adapter_device_index, adapter_passive),
                    on_disconnected=lambda: self._mark_imu_disconnected(slot, address, transport, adapter_port, adapter_device_index, adapter_passive),
                    aux_poll=False,
                )
                task = asyncio.create_task(client.run(duration_s=None))
                while not task.done():
                    if stop_event.is_set():
                        task.cancel()
                        break
                    await asyncio.sleep(0.1)
                try:
                    await task
                except asyncio.CancelledError:
                    return

            if transport == "serial_adapter":
                while not stop_event.is_set():
                    try:
                        client = WT901SerialAdapterClient(
                            adapter_port or "",
                            sensor_id,
                            lambda sample: self._handle_imu_sample(slot, sample, address, transport, adapter_port, adapter_device_index, adapter_passive),
                            on_connected=lambda: self._mark_imu_waiting_data(slot, address, transport, adapter_port, adapter_device_index, adapter_passive),
                            on_status=lambda message: self._mark_imu_status(slot, address, transport, adapter_port, adapter_device_index, adapter_passive, message),
                            address=address,
                            device_index=adapter_device_index,
                            passive=adapter_passive,
                            aux_poll=False,
                        )
                        client.run(duration_s=None, should_stop=stop_event.is_set)
                    except Exception as exc:
                        if stop_event.is_set():
                            break
                        with self.lock:
                            if self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                                self.imu_slots[slot].connection_state = "reconnecting"
                                self.imu_slots[slot].last_error = f"{exc}\n{traceback.format_exc()}"
                        time.sleep(1.0)
            else:
                asyncio.run(run_client())
            with self.lock:
                if self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                    self.imu_slots[slot].connection_state = "disconnected"
                    self.imu_slots[slot].recording = False
                    self._imu_record_paths[slot] = None
        except Exception as exc:
            with self.lock:
                if self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                    self.imu_slots[slot].connection_state = "failed"
                    self.imu_slots[slot].recording = False
                    self.imu_slots[slot].last_error = f"{exc}\n{traceback.format_exc()}"
                    self._imu_record_paths[slot] = None

    def _is_current_imu_locked(
        self,
        slot: str,
        address: str,
        transport: str,
        adapter_port: str | None,
        adapter_device_index: int | None,
        adapter_passive: bool,
    ) -> bool:
        status = self.imu_slots[slot]
        return (
            status.selected_address == address
            and status.transport == transport
            and status.adapter_port == adapter_port
            and status.adapter_device_index == adapter_device_index
            and status.adapter_passive == adapter_passive
        )

    def _mark_imu_connected(self, slot: str, address: str, transport: str, adapter_port: str | None, adapter_device_index: int | None, adapter_passive: bool) -> None:
        with self.lock:
            if not self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                return
            self.imu_slots[slot].connection_state = "connected"
            self.imu_slots[slot].connection_monotonic_ns = time.monotonic_ns()
            self.imu_slots[slot].last_error = None

    def _mark_imu_waiting_data(self, slot: str, address: str, transport: str, adapter_port: str | None, adapter_device_index: int | None, adapter_passive: bool) -> None:
        with self.lock:
            if not self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                return
            self.imu_slots[slot].connection_state = "waiting_data"
            self.imu_slots[slot].connection_monotonic_ns = time.monotonic_ns()
            self.imu_slots[slot].last_error = None

    def _mark_imu_status(self, slot: str, address: str, transport: str, adapter_port: str | None, adapter_device_index: int | None, adapter_passive: bool, message: str) -> None:
        with self.lock:
            if not self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                return
            self.imu_slots[slot].connection_state = "connecting"
            self.imu_slots[slot].connection_monotonic_ns = time.monotonic_ns()
            self.imu_slots[slot].last_error = message

    def _mark_imu_disconnected(self, slot: str, address: str, transport: str, adapter_port: str | None, adapter_device_index: int | None, adapter_passive: bool) -> None:
        with self.lock:
            if not self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                return
            self.imu_slots[slot].connection_state = "disconnected"
            self.imu_slots[slot].recording = False
            self.imu_slots[slot].last_error = "BLE disconnected."
            self._imu_record_paths[slot] = None

    def _handle_imu_sample(self, slot: str, sample, address: str, transport: str, adapter_port: str | None, adapter_device_index: int | None, adapter_passive: bool) -> None:
        path = None
        live_path = None
        with self.lock:
            if not self._is_current_imu_locked(slot, address, transport, adapter_port, adapter_device_index, adapter_passive):
                return
            self.imu_slots[slot].connection_state = "connected"
            self.imu_slots[slot].connection_monotonic_ns = time.monotonic_ns()
            self.imu_slots[slot].last_error = None
            self.imu_slots[slot].last_sample_monotonic_ns = sample.timestamp_monotonic_ns
            self.imu_slots[slot].last_sample = asdict(sample)
            self.imu_slots[slot].sample_count += 1
            path = self._imu_record_paths.get(slot)
            live_path = self._imu_live_paths.get(slot)
        if path is not None:
            write_jsonl_sample(path, sample)
        if live_path is not None:
            write_jsonl_sample(live_path, sample)

    def _refresh_imu_health_locked(self) -> None:
        now_ns = time.monotonic_ns()
        for status in self.imu_slots.values():
            if status.connection_state == "waiting_data" and status.connection_monotonic_ns is not None:
                age_s = (now_ns - status.connection_monotonic_ns) / 1_000_000_000
                if age_s >= IMU_NO_DATA_WARN_S and status.sample_count == 0:
                    status.connection_state = "no_data"
                    status.last_error = (
                        "Adapter connected but no WT901 data packets were received. "
                        "Check the selected /dev/ttyACM port, close any BLE direct connection, "
                        "and make sure no other process owns the adapter."
                    )
            if status.connection_state == "connected" and status.last_sample_monotonic_ns is not None:
                age_s = (now_ns - status.last_sample_monotonic_ns) / 1_000_000_000
                if age_s >= IMU_STALE_SAMPLE_WARN_S:
                    status.connection_state = "stale"
                    status.last_error = f"No IMU sample received for {age_s:.1f}s."


class DashboardServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], jobs: CaptureJobs) -> None:
        super().__init__(address, DashboardHandler)
        self.jobs = jobs
        self.preview_manager = PreviewCaptureManager()
        self.active_lock = threading.Lock()
        self.active_streams: dict[object, str] = {}

    def activate_stream(self, source: object, fourcc: str, width: int, height: int, fps: float) -> None:
        with self.active_lock:
            self.active_streams[source] = _stream_key(source, fourcc, width, height, fps)

    def is_active_stream(self, source: object, fourcc: str, width: int, height: int, fps: float) -> bool:
        with self.active_lock:
            return self.active_streams.get(source) == _stream_key(source, fourcc, width, height, fps)

    def active_stream_snapshot(self) -> dict[str, str]:
        with self.active_lock:
            return dict(self.active_streams)

    def stop_previews(self) -> None:
        with self.active_lock:
            self.active_streams.clear()
        self.preview_manager.stop_all()


class DashboardHandler(BaseHTTPRequestHandler):
    server: DashboardServer

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_html(INDEX_HTML)
            return
        if parsed.path == "/api/cameras":
            include_configs = parse_qs(parsed.query).get("configs", ["0"])[0] == "1"
            rows = []
            for device in find_capture_devices():
                row = device.as_dict()
                if include_configs:
                    row["configs"] = get_camera_config(device.path)
                rows.append(row)
            self._send_json(rows)
            return
        if parsed.path == "/api/status":
            self._send_json(self.server.jobs.snapshot())
            return
        if parsed.path == "/api/preview/status":
            self._send_json({"active_streams": self.server.active_stream_snapshot(), "captures": self.server.preview_manager.snapshot()})
            return
        if parsed.path == "/api/record/latest-frame":
            params = parse_qs(parsed.query)
            camera_id = params.get("camera_id", [""])[0]
            self._send_latest_frame(camera_id)
            return
        if parsed.path == "/api/camera/profile":
            self._send_json(
                {
                    "width": DEFAULT_CAMERA_WIDTH,
                    "height": DEFAULT_CAMERA_HEIGHT,
                    "fps": DEFAULT_CAMERA_FPS,
                    "format": DEFAULT_CAMERA_FORMAT,
                    "preview_fps": DEFAULT_PREVIEW_FPS,
                }
            )
            return
        if parsed.path == "/api/imu/scan":
            timeout_s = _optional_float(parse_qs(parsed.query).get("timeout_s", [str(DEFAULT_IMU_SCAN_TIMEOUT_S)])[0]) or DEFAULT_IMU_SCAN_TIMEOUT_S
            try:
                devices = _scan_imu_devices(timeout_s)
            except RuntimeError as exc:
                self._send_json({"error": str(exc)}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                return
            except Exception as exc:
                self._send_json({"error": f"IMU scan failed: {exc}"}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            self._send_json(devices)
            return
        if parsed.path == "/api/imu/known":
            self._send_json(KNOWN_IMU_DEVICES)
            return
        if parsed.path == "/stream":
            params = parse_qs(parsed.query)
            source = unquote(params.get("source", params.get("device", [""]))[0])
            fourcc = unquote(params.get("format", [""])[0])
            width = _optional_int(params.get("width", [None])[0])
            height = _optional_int(params.get("height", [None])[0])
            device_fps = _optional_float(params.get("device_fps", [None])[0])
            output_fps = _optional_float(params.get("output_fps", [None])[0]) or DEFAULT_PREVIEW_FPS
            self._stream_camera(source, fourcc, width, height, device_fps, output_fps)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        payload = self._read_json()
        try:
            if parsed.path == "/api/camera/start":
                self.server.stop_previews()
                self._send_json(self.server.jobs.start_record(payload).as_dict())
                return
            if parsed.path == "/api/camera/stop":
                self._send_json(self.server.jobs.stop_record().as_dict())
                return
            if parsed.path == "/api/record/start":
                self.server.stop_previews()
                self._send_json(self.server.jobs.start_record(payload).as_dict())
                return
            if parsed.path == "/api/record/stop":
                self._send_json(self.server.jobs.stop_record().as_dict())
                return
            if parsed.path == "/api/preview/stop":
                self.server.stop_previews()
                self._send_json({"ok": True})
                return
            if parsed.path == "/api/imu/start":
                self._send_json(self.server.jobs.select_imu(payload).as_dict())
                return
            if parsed.path == "/api/imu/stop":
                self._send_json(self.server.jobs.disconnect_imu(_imu_slot(payload)).as_dict())
                return
            if parsed.path == "/api/imu/select":
                self._send_json(self.server.jobs.select_imu(payload).as_dict())
                return
            if parsed.path == "/api/visualization/start":
                self.server.stop_previews()
                self._send_json(self.server.jobs.start_visualization(payload).as_dict())
                return
            if parsed.path == "/api/visualization/stop":
                self._send_json(self.server.jobs.stop_visualization().as_dict())
                return
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def _stream_camera(
        self,
        source: str,
        fourcc: str,
        width: int | None,
        height: int | None,
        device_fps: float | None,
        output_fps: float,
    ) -> None:
        if not source:
            self.send_error(HTTPStatus.BAD_REQUEST, "Missing source")
            return
        fourcc = _normalize_fourcc(fourcc)
        width = width or DEFAULT_CAMERA_WIDTH
        height = height or DEFAULT_CAMERA_HEIGHT
        capture_fps = device_fps or output_fps or DEFAULT_PREVIEW_FPS
        capture_source = _normalize_camera_source(source)
        if not _is_valid_capture_source(capture_source):
            self.send_error(HTTPStatus.BAD_REQUEST, f"Invalid camera source: {source}")
            return
        self.server.activate_stream(capture_source, fourcc, width, height, capture_fps)
        try:
            import cv2  # noqa: F401

            self.server.preview_manager.get(capture_source, fourcc, width, height, capture_fps)
        except ImportError:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "opencv-python is not installed")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        delay = 1.0 / max(1.0, min(output_fps, 15.0))
        try:
            while self.server.is_active_stream(capture_source, fourcc, width, height, capture_fps):
                frame = self.server.preview_manager.read_jpeg(capture_source, fourcc, width, height, capture_fps)
                if frame is None:
                    frame = _make_placeholder_jpeg(source)
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(delay)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _send_latest_frame(self, camera_id: str) -> None:
        if camera_id not in {"C0", "C1", "C2", "C3"}:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid camera_id")
            return
        session_dir = self.server.jobs._session_dir
        if session_dir is None:
            self.send_error(HTTPStatus.NOT_FOUND, "No active or recent session")
            return
        camera_dir = session_dir / "cameras" / camera_id
        try:
            image_path = max(camera_dir.glob("*.jpg"), key=lambda path: path.stat().st_mtime_ns)
        except ValueError:
            self.send_error(HTTPStatus.NOT_FOUND, "No frame available yet")
            return
        body = image_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def _parse_source(source: str):
    return _normalize_camera_source(source)


def _normalize_camera_source(source: str):
    value = str(source or "").strip()
    match = re.match(r"^C[0-3]:(.+)$", value)
    if match:
        value = match.group(1).strip()
    path_match = re.search(r"(/dev/video\d+)\b", value)
    if path_match:
        return path_match.group(1)
    video_match = re.search(r"\b(video\d+)\b", value)
    if video_match:
        return f"/dev/{video_match.group(1)}"
    if re.fullmatch(r"\d+", value):
        return int(value)
    return value


def _is_valid_capture_source(source: object) -> bool:
    if isinstance(source, int):
        return source >= 0
    value = str(source or "").strip()
    return value.startswith("/dev/")


def _normalize_fourcc(value: str) -> str:
    value = (value or "MJPG").upper()[:4]
    return value.ljust(4)


def _stream_key(source: object, fourcc: str, width: int, height: int, fps: float) -> str:
    return f"{source}|{_normalize_fourcc(fourcc)}|{width}x{height}@{fps:g}"


def _make_placeholder_jpeg(label: str) -> bytes:
    import cv2

    canvas = cv2.UMat(720, 1280, cv2.CV_8UC3).get()
    canvas[:] = (18, 20, 28)
    cv2.putText(canvas, f"No signal: {label}", (60, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (180, 190, 205), 2)
    ok, encoded = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return encoded.tobytes() if ok else b""


def _imu_device_key(device: dict) -> str:
    return "|".join(
        [
            str(device.get("transport") or "ble"),
            str(device.get("adapter_port") or ""),
            str(device.get("address") or ""),
        ]
    )


def _scan_imu_devices(timeout_s: float) -> list[dict[str, str]]:
    devices_by_address = {_imu_device_key(device): dict(device) for device in KNOWN_IMU_DEVICES}
    for port in sorted({DEFAULT_HEAD_IMU_ADAPTER_PORT, DEFAULT_WRIST_IMU_ADAPTER_PORT}):
        try:
            for device in scan_serial_adapter_devices(port, timeout_s=min(timeout_s, 8.0)):
                dashboard_device = {
                    "name": device.name,
                    "address": device.address,
                    "transport": "serial_adapter",
                    "adapter_port": port,
                    "adapter_device_index": device.index,
                    "adapter_passive": False,
                }
                known = _known_imu_device_by_address(device.address)
                if known is not None:
                    dashboard_device["slot"] = known.get("slot")
                    dashboard_device["name"] = known.get("name") or dashboard_device["name"]
                devices_by_address[_imu_device_key(dashboard_device)] = dashboard_device
        except Exception:
            continue
    for name, address in asyncio.run(scan_wt_devices(timeout_s)):
        device = {"name": name, "address": address}
        devices_by_address[_imu_device_key(device)] = device
    return list(devices_by_address.values())


def _known_imu_device_by_address(address: str) -> dict | None:
    wanted = address.lower()
    for device in KNOWN_IMU_DEVICES:
        if str(device.get("address") or "").lower() == wanted:
            return device
    return None


def _optional_int(value) -> int | None:
    if value in (None, ""):
        return None
    return int(value)


def _optional_float(value) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _optional_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _camera_sources_from_payload(payload: dict) -> list[CameraSource]:
    sources = []
    camera_configs = payload.get("camera_configs") or {}
    for index in range(4):
        camera_id = f"C{index}"
        raw_source = str(payload.get(camera_id, "")).strip()
        if raw_source:
            config = camera_configs.get(camera_id) or {}
            fourcc = str(config.get("format") or DEFAULT_CAMERA_FORMAT).strip() or None
            sources.append(CameraSource(camera_id=camera_id, source=_parse_source(raw_source), fourcc=fourcc))
    return sources


def _record_profile_from_payload(payload: dict) -> dict:
    return {
        "width": _optional_int(payload.get("width")) or DEFAULT_CAMERA_WIDTH,
        "height": _optional_int(payload.get("height")) or DEFAULT_CAMERA_HEIGHT,
        "fps": _optional_float(payload.get("fps")) or DEFAULT_CAMERA_FPS,
    }


def _shell_quote(value: str) -> str:
    return "'" + str(value).replace("'", "'\"'\"'") + "'"


def _imu_slot(payload: dict) -> str:
    slot = str(payload.get("slot") or "wrist_imu").strip()
    if slot not in {"head_imu", "wrist_imu"}:
        raise RuntimeError(f"Unknown IMU slot: {slot}")
    return slot


def _apply_serial_adapter_defaults(slot_ports: dict[str, str | None]) -> None:
    for device in KNOWN_IMU_DEVICES:
        slot = str(device.get("slot") or "")
        port = slot_ports.get(slot)
        if not port:
            continue
        device["transport"] = "serial_adapter"
        device["adapter_port"] = port


def _session_dir(payload: dict) -> Path:
    root = Path(str(payload.get("session_root") or "data/raw"))
    label = datetime.now().strftime("session_%Y%m%d_%H%M%S")
    return root / label


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>3D Motion Capture Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0d12;
      --panel: #151922;
      --line: #2b3242;
      --text: #eef2f7;
      --muted: #9aa4b5;
      --accent: #3b82f6;
      --good: #22c55e;
      --bad: #ef4444;
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text); font-family: Inter, system-ui, sans-serif; }
    header { padding: 14px 18px; border-bottom: 1px solid var(--line); display: flex; justify-content: space-between; gap: 16px; align-items: center; }
    h1 { margin: 0; font-size: 18px; letter-spacing: 0; }
    main { display: grid; grid-template-columns: minmax(0, 1fr); gap: 14px; padding: 14px; align-items: start; }
    section { background: var(--panel); border: 1px solid var(--line); padding: 12px; }
    h2 { margin: 0 0 10px; font-size: 15px; }
    label { display: grid; gap: 5px; color: var(--muted); font-size: 12px; margin-bottom: 9px; }
    input, select, button { width: 100%; border: 1px solid var(--line); background: #0f131b; color: var(--text); padding: 8px; font: inherit; }
    button { cursor: pointer; background: #1d2430; }
    button.primary { background: var(--accent); border-color: var(--accent); font-weight: 700; }
    button.stop { background: var(--bad); border-color: var(--bad); font-weight: 700; }
    .controls-panel { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 14px; align-items: start; }
    .controls-panel section { margin-top: 0 !important; min-height: 100%; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    .camera-toolbar { display: grid; grid-template-columns: minmax(0, 1fr) 180px; gap: 10px; align-items: start; margin-bottom: 10px; }
    .camera-toolbar h2 { margin: 0; }
    .camera-toolbar label { margin: 0; }
    .preview-section { min-width: 0; overflow-x: auto; }
    .camera-grid { display: grid; grid-template-columns: repeat(var(--camera-columns, 2), minmax(0, 1fr)); gap: 10px; }
    .camera-grid[data-columns="4"] { grid-template-columns: repeat(4, minmax(420px, 1fr)); min-width: calc(4 * 420px + 3 * 10px); }
    .camera-grid[data-columns="4"] .tile { min-height: calc(100vh - 130px); }
    .camera-grid[data-columns="4"] .config-row { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .camera-grid[data-columns="4"] .screen { min-height: 0; }
    .tile { border: 1px solid var(--line); background: #05070b; min-height: 300px; display: grid; grid-template-rows: auto auto minmax(0, 1fr); }
    .tile-head { display: grid; grid-template-columns: auto minmax(0, 1fr) auto; gap: 8px; align-items: center; padding: 8px; border-bottom: 1px solid var(--line); }
    .view-title { color: var(--muted); font-size: 12px; font-weight: 700; }
    .badge { color: var(--muted); border: 1px solid var(--line); padding: 4px 7px; font-size: 11px; }
    .badge.live { color: var(--good); border-color: var(--good); }
    .badge.offline { color: var(--bad); border-color: var(--bad); }
    .config-row { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 6px; padding: 8px; border-bottom: 1px solid var(--line); }
    .field { margin: 0; }
    .screen { position: relative; min-height: 220px; overflow: hidden; display: grid; place-items: center; background: #05070b; }
    .screen img { max-width: 100%; max-height: 100%; transform: rotate(var(--rotation, 0deg)); }
    .overlay { position: absolute; left: 8px; right: 8px; bottom: 8px; padding: 6px 8px; background: rgba(5, 7, 11, 0.75); color: var(--muted); font-size: 12px; }
    .imu-data { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; margin-top: 8px; }
    .metric { border: 1px solid var(--line); background: #0f131b; padding: 8px; }
    .metric strong { display: block; color: var(--muted); font-size: 11px; margin-bottom: 5px; }
    .metric span { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; }
    .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 8px; }
    pre { white-space: pre-wrap; color: var(--muted); background: #0f131b; border: 1px solid var(--line); padding: 10px; min-height: 80px; max-height: 220px; overflow: auto; }
    .status { color: var(--muted); font-size: 13px; }
    .status b { color: var(--good); }
    @media (max-width: 1100px) {
      .controls-panel { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>3D Motion Capture Dashboard</h1>
    <div class="status" id="status">加载中</div>
  </header>
  <main>
    <div class="controls-panel">
      <section>
        <h2>Camera 设置</h2>
        <div class="row">
          <button id="refreshCameras">刷新设备</button>
          <button id="assignCameras">自动分配前四个</button>
        </div>
        <div class="status" id="cameraProfile">每路相机可单独选择 format / size / fps。</div>
        <label>Session root <input id="session_root" value="data/raw"></label>
        <button class="primary" id="recordToggle">开始 Record</button>
      </section>
      <section style="margin-top: 14px;">
        <h2>IMU 设置</h2>
        <button id="scanImu">扫描 IMU</button>
        <div class="actions">
          <button id="connectAllImu">连接两个 IMU</button>
          <button id="disconnectAllImu">断开两个 IMU</button>
        </div>
        <h2>Head IMU</h2>
        <label>Device <select id="head_imu_device"></select></label>
        <div class="actions">
          <button id="head_imu_connect">连接所选</button>
          <button id="head_imu_disconnect">断开</button>
        </div>
        <div class="status" id="head_imu_status">未选择</div>
        <h2 style="margin-top: 14px;">Wrist IMU</h2>
        <label>Device <select id="wrist_imu_device"></select></label>
        <div class="actions">
          <button id="wrist_imu_connect">连接所选</button>
          <button id="wrist_imu_disconnect">断开</button>
        </div>
        <div class="status" id="wrist_imu_status">未选择</div>
      </section>
      <section style="margin-top: 14px;">
        <h2>状态</h2>
        <label><input id="enableHands" type="checkbox"> 手部骨骼</label>
        <button id="visualizeToggle">启动 RViz 可视化</button>
        <pre id="log"></pre>
      </section>
    </div>
    <section class="preview-section">
      <div class="camera-toolbar">
        <h2>Camera 预览</h2>
        <label>Layout
          <select id="cameraLayout">
            <option value="2">2 columns</option>
            <option value="4">4 columns</option>
            <option value="1">1 column</option>
            <option value="3">3 columns</option>
          </select>
        </label>
      </div>
      <div class="camera-grid" id="cameraGrid"></div>
    </section>
  </main>
  <script>
    const SLOT_COUNT = 4;
    const cameraIds = ['C0', 'C1', 'C2', 'C3'];
    const imuSlots = ['head_imu', 'wrist_imu'];
    const PREFERRED_FORMAT = 'MJPG';
    const PREFERRED_SIZE = '1600x1200';
    const PREFERRED_FPS = 25;
    const SOFTWARE_FPS_OPTIONS = [30, 25, 15, 10, 5];
    const $ = id => document.getElementById(id);
    const cameraGrid = $('cameraGrid');
    const cameraLayout = $('cameraLayout');
    let cameraProfile = {width: 1600, height: 1200, fps: 25, format: 'MJPG', preview_fps: 8};
    let cameras = [];
    let configs = {};
    let selections = Array.from({length: SLOT_COUNT}, () => '');
    let streamConfigs = Array.from({length: SLOT_COUNT}, () => ({format: '', size: '', fps: ''}));
    let rotations = Array.from({length: SLOT_COUNT}, () => 0);
    let imuDevices = [];
    let recordingActive = false;
    let visualizationActive = false;
    let recordingPreviewTimer = null;

    function applyCameraLayout() {
      const columns = Number(cameraLayout.value || 2);
      const safeColumns = [1, 2, 3, 4].includes(columns) ? columns : 2;
      cameraGrid.style.setProperty('--camera-columns', String(safeColumns));
      cameraGrid.dataset.columns = String(safeColumns);
    }

    async function api(path, options = {}) {
      const res = await fetch(path, options);
      const text = await res.text();
      const data = text ? JSON.parse(text) : {};
      if (!res.ok) throw new Error(data.error || res.statusText);
      return data;
    }

    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, char => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
      }[char]));
    }

    function showError(error) {
      const message = error?.stack || error?.message || String(error);
      $('status').innerHTML = `<b style="color: var(--bad);">Error</b>`;
      $('log').textContent = message;
    }

    async function runAction(action) {
      try { await action(); } catch (error) { showError(error); }
    }

    function cameraLabel(path) {
      const camera = cameras.find(item => item.path === path);
      return camera ? camera.label : path || 'No camera selected';
    }

    function optionsHtml() {
      return '<option value="">No camera</option>' +
        cameras.map(camera => `<option value="${escapeHtml(camera.path)}">${escapeHtml(camera.label)}</option>`).join('');
    }

    function formatOptions(path, selectedFormat = '') {
      return (configs[path] || []).map(format => {
        const selected = format.format === selectedFormat ? ' selected' : '';
        return `<option value="${escapeHtml(format.format)}"${selected}>${escapeHtml(format.format)} - ${escapeHtml(format.description || '')}</option>`;
      }).join('');
    }

    function selectedFormat(path, slot) {
      const formats = configs[path] || [];
      const current = streamConfigs[slot]?.format;
      return formats.find(format => format.format === current)?.format ||
        formats.find(format => format.format === (cameraProfile.format || PREFERRED_FORMAT))?.format ||
        formats.find(format => format.format === PREFERRED_FORMAT)?.format ||
        formats[0]?.format || '';
    }

    function selectedSize(path, slot, formatName) {
      const format = (configs[path] || []).find(item => item.format === formatName);
      const current = streamConfigs[slot]?.size;
      const preferred = `${cameraProfile.width || 1600}x${cameraProfile.height || 1200}`;
      return format?.sizes?.find(size => (size.label || `${size.width}x${size.height}`) === current)?.label ||
        format?.sizes?.find(size => (size.label || `${size.width}x${size.height}`) === preferred)?.label ||
        format?.sizes?.find(size => (size.label || `${size.width}x${size.height}`) === PREFERRED_SIZE)?.label ||
        (format?.sizes?.[0] ? (format.sizes[0].label || `${format.sizes[0].width}x${format.sizes[0].height}`) : '');
    }

    function selectedFps(path, slot, formatName, sizeLabel) {
      const format = (configs[path] || []).find(item => item.format === formatName);
      const size = format?.sizes?.find(item => (item.label || `${item.width}x${item.height}`) === sizeLabel);
      const current = Number(streamConfigs[slot]?.fps || 0);
      const options = fpsValues(size);
      if (options.includes(current)) return String(current);
      const target = Number(cameraProfile.fps || PREFERRED_FPS);
      const preferred = options.find(value => value === target) ||
        options.find(value => value <= target) ||
        options[options.length - 1];
      return String(preferred || '');
    }

    function hardwareFpsValues(size) {
      return (size?.fps || []).map(Number).filter(value => value > 0).sort((a, b) => b - a);
    }

    function fpsValues(size) {
      const hardware = hardwareFpsValues(size);
      const maxHardware = Math.max(0, ...hardware);
      const software = SOFTWARE_FPS_OPTIONS.filter(value => maxHardware <= 0 || value < maxHardware);
      return Array.from(new Set([...hardware, ...software])).sort((a, b) => b - a);
    }

    function selectedSizeConfig(path, formatName, sizeLabel) {
      const format = (configs[path] || []).find(item => item.format === formatName);
      return format?.sizes?.find(item => (item.label || `${item.width}x${item.height}`) === sizeLabel);
    }

    function deviceFpsFor(size, requestedFps) {
      const requested = Number(requestedFps || 0);
      const hardware = hardwareFpsValues(size);
      if (!hardware.length) return requested || '';
      if (hardware.includes(requested)) return requested;
      return hardware.slice().reverse().find(value => value >= requested) || hardware[hardware.length - 1];
    }

    function sizeOptions(path, formatName, selectedSizeLabel = '') {
      const format = (configs[path] || []).find(item => item.format === formatName);
      return (format?.sizes || []).map(size => {
        const label = size.label || `${size.width}x${size.height}`;
        const selected = label === selectedSizeLabel ? ' selected' : '';
        return `<option value="${escapeHtml(label)}"${selected}>${escapeHtml(label)}</option>`;
      }).join('');
    }

    function fpsOptions(path, formatName, sizeLabel, selectedFpsValue = '') {
      const size = selectedSizeConfig(path, formatName, sizeLabel);
      const hardware = new Set(hardwareFpsValues(size));
      return fpsValues(size).map(fps => {
        const selected = String(fps) === String(selectedFpsValue) ? ' selected' : '';
        const suffix = hardware.has(Number(fps)) ? '' : ' software';
        return `<option value="${fps}"${selected}>${fps} fps${suffix}</option>`;
      }).join('');
    }

    function rotationOptions(selectedRotation = 0) {
      return [0, 90, 180, 270].map(angle => {
        const selected = Number(selectedRotation) === angle ? ' selected' : '';
        return `<option value="${angle}"${selected}>${angle} deg</option>`;
      }).join('');
    }

    function normalizeSlotConfig(slot) {
      const path = selections[slot] || '';
      const format = selectedFormat(path, slot);
      const size = selectedSize(path, slot, format);
      const fps = selectedFps(path, slot, format, size);
      const device_fps = deviceFpsFor(selectedSizeConfig(path, format, size), fps);
      streamConfigs[slot] = {format, size, fps, device_fps};
      return streamConfigs[slot];
    }

    function streamUrl(path, config = {}) {
      if (!path) return '';
      const params = new URLSearchParams({source: path, t: Date.now()});
      if (config.format) params.set('format', config.format);
      if (config.size) {
        const [width, height] = config.size.split('x');
        params.set('width', width);
        params.set('height', height);
      }
      if (config.device_fps) params.set('device_fps', config.device_fps);
      if (config.fps) params.set('output_fps', Math.min(Number(config.fps), Number(cameraProfile.preview_fps)));
      return `/stream?${params.toString()}`;
    }

    function applyRotation(img, angle) {
      img.style.setProperty('--rotation', `${Number(angle || 0) % 360}deg`);
    }

    function renderCameraGrid() {
      cameraGrid.innerHTML = '';
      for (let i = 0; i < SLOT_COUNT; i += 1) {
        const selected = selections[i] || '';
        const config = normalizeSlotConfig(i);
        const tile = document.createElement('article');
        tile.className = 'tile';
        tile.innerHTML = `
          <div class="tile-head">
            <div class="view-title">${cameraIds[i]}</div>
            <select aria-label="Camera for ${cameraIds[i]}">${optionsHtml()}</select>
            <div class="badge ${selected ? 'live' : 'idle'}">${selected ? 'LIVE' : 'IDLE'}</div>
          </div>
          <div class="config-row">
            <label class="field"><span>Format</span><select class="format-select">${formatOptions(selected, config.format)}</select></label>
            <label class="field"><span>Size</span><select class="size-select">${sizeOptions(selected, config.format, config.size)}</select></label>
            <label class="field"><span>FPS</span><select class="fps-select">${fpsOptions(selected, config.format, config.size, config.fps)}</select></label>
            <label class="field"><span>Rotate</span><select class="rotate-select">${rotationOptions(rotations[i])}</select></label>
          </div>
          <div class="screen">
            <img src="${streamUrl(selected, config)}" alt="${cameraIds[i]} preview">
            <div class="overlay">${cameraLabel(selected)}${config.size ? ` | ${config.format} ${config.size} @ ${config.fps}fps` : ''}</div>
          </div>`;
        const cameraSelect = tile.querySelector('.tile-head select');
        const formatSelect = tile.querySelector('.format-select');
        const sizeSelect = tile.querySelector('.size-select');
        const fpsSelect = tile.querySelector('.fps-select');
        const rotateSelect = tile.querySelector('.rotate-select');
        const img = tile.querySelector('img');
        const badge = tile.querySelector('.badge');
        const overlay = tile.querySelector('.overlay');
        cameraSelect.value = selected;

        const updateControls = () => {
          const cfg = normalizeSlotConfig(i);
          formatSelect.innerHTML = formatOptions(selections[i], cfg.format);
          sizeSelect.innerHTML = sizeOptions(selections[i], cfg.format, cfg.size);
          fpsSelect.innerHTML = fpsOptions(selections[i], cfg.format, cfg.size, cfg.fps);
          formatSelect.value = cfg.format;
          sizeSelect.value = cfg.size;
          fpsSelect.value = cfg.fps;
          return cfg;
        };
        const restartStream = () => {
          const cfg = normalizeSlotConfig(i);
          const nextUrl = streamUrl(selections[i], cfg);
          img.removeAttribute('src');
          requestAnimationFrame(() => {
            img.src = nextUrl;
          });
          overlay.textContent = `${cameraLabel(selections[i])}${cfg.size ? ` | ${cfg.format} ${cfg.size} @ ${cfg.fps}fps` : ''}`;
          badge.textContent = selections[i] ? 'LIVE' : 'IDLE';
          badge.className = `badge ${selections[i] ? 'live' : 'idle'}`;
          applyRotation(img, rotations[i]);
        };
        cameraSelect.addEventListener('change', () => {
          selections[i] = cameraSelect.value;
          streamConfigs[i] = {format: '', size: '', fps: '', device_fps: ''};
          updateControls();
          restartStream();
        });
        formatSelect.addEventListener('change', () => {
          streamConfigs[i].format = formatSelect.value;
          streamConfigs[i].size = '';
          streamConfigs[i].fps = '';
          streamConfigs[i].device_fps = '';
          updateControls();
          restartStream();
        });
        sizeSelect.addEventListener('change', () => {
          streamConfigs[i].size = sizeSelect.value;
          streamConfigs[i].fps = '';
          streamConfigs[i].device_fps = '';
          updateControls();
          restartStream();
        });
        fpsSelect.addEventListener('change', () => {
          streamConfigs[i].fps = fpsSelect.value;
          restartStream();
        });
        rotateSelect.addEventListener('change', () => {
          rotations[i] = Number(rotateSelect.value || 0);
          applyRotation(img, rotations[i]);
        });
        img.addEventListener('error', () => {
          badge.textContent = selections[i] ? 'OFFLINE' : 'IDLE';
          badge.className = `badge ${selections[i] ? 'offline' : 'idle'}`;
        });
        applyRotation(img, rotations[i]);
        cameraGrid.appendChild(tile);
      }
    }

    function clearCameraPreviews() {
      cameraGrid.querySelectorAll('.screen img').forEach(img => img.removeAttribute('src'));
    }

    function restartCameraPreviews() {
      stopRecordingPreviews();
      cameraGrid.querySelectorAll('.tile').forEach((tile, index) => {
        const img = tile.querySelector('.screen img');
        const cfg = normalizeSlotConfig(index);
        img.src = streamUrl(selections[index], cfg);
        const badge = tile.querySelector('.badge');
        badge.textContent = selections[index] ? 'LIVE' : 'IDLE';
        badge.className = `badge ${selections[index] ? 'live' : 'idle'}`;
      });
    }

    function latestFrameUrl(cameraId) {
      return `/api/record/latest-frame?camera_id=${encodeURIComponent(cameraId)}&t=${Date.now()}`;
    }

    function updateRecordingPreviews() {
      cameraGrid.querySelectorAll('.tile').forEach((tile, index) => {
        if (!selections[index]) return;
        const img = tile.querySelector('.screen img');
        const badge = tile.querySelector('.badge');
        img.src = latestFrameUrl(cameraIds[index]);
        badge.textContent = 'REC';
        badge.className = 'badge live';
      });
    }

    function startRecordingPreviews() {
      stopRecordingPreviews();
      updateRecordingPreviews();
      recordingPreviewTimer = setInterval(updateRecordingPreviews, 500);
    }

    function stopRecordingPreviews() {
      if (recordingPreviewTimer !== null) {
        clearInterval(recordingPreviewTimer);
        recordingPreviewTimer = null;
      }
    }

    function selectedRecordConfig() {
      const index = selections.findIndex(Boolean);
      const config = index >= 0 ? normalizeSlotConfig(index) : {size: `${cameraProfile.width}x${cameraProfile.height}`, fps: String(cameraProfile.fps)};
      const [width, height] = (config.size || `${cameraProfile.width}x${cameraProfile.height}`).split('x').map(Number);
      return {width, height, fps: config.fps || cameraProfile.fps};
    }

    function recordPayload() {
      const cfg = selectedRecordConfig();
      const data = {session_root: $('session_root').value, width: cfg.width, height: cfg.height, fps: cfg.fps, hands: Boolean($('enableHands')?.checked)};
      selections.forEach((source, index) => {
        if (source) data[cameraIds[index]] = source;
      });
      data.camera_configs = Object.fromEntries(selections.map((source, index) => [cameraIds[index], {source, ...streamConfigs[index]}]));
      return data;
    }

    async function loadProfile() {
      cameraProfile = await api('/api/camera/profile');
    }

    async function loadCameras(assignMissing = false) {
      const rows = await api('/api/cameras?configs=1');
      cameras = rows;
      configs = Object.fromEntries(rows.map(row => [row.path, row.configs || []]));
      const valid = new Set(cameras.map(camera => camera.path));
      selections = selections.map(source => valid.has(source) ? source : '');
      if (assignMissing) {
        for (let i = 0; i < SLOT_COUNT; i += 1) {
          if (!selections[i] && cameras[i]) selections[i] = cameras[i].path;
        }
      }
      renderCameraGrid();
      $('cameraProfile').textContent = `${cameras.length} cameras detected. Record uses the first active camera tile's size/fps as the session capture profile.`;
    }

    function imuLabel(device) {
      const transport = device.transport === 'serial_adapter' ? ` via ${device.adapter_port}` : '';
      const mode = device.adapter_passive ? ' passive' : '';
      return `${device.name || 'WT IMU'} ${device.address}${transport}${mode}`;
    }

    function imuKey(device) {
      return `${device.transport || 'ble'}|${device.adapter_port || ''}|${device.address || ''}|${device.adapter_passive ? 'passive' : 'active'}`;
    }

    function mergeImuDevices(devices) {
      const byKey = new Map(imuDevices.map(device => [imuKey(device), device]));
      for (const device of devices || []) {
        const normalized = {...device, transport: device.transport || 'ble'};
        byKey.set(imuKey(normalized), {...byKey.get(imuKey(normalized)), ...normalized});
      }
      imuDevices = [...byKey.values()];
    }

    function defaultImuForSlot(slot) {
      return imuDevices.find(device => device.slot === slot);
    }

    function populateImuSelect(slot, selected = '') {
      const select = $(slot + '_device');
      select.innerHTML = '<option value="">No IMU</option>';
      for (const device of imuDevices) {
        const opt = document.createElement('option');
        opt.value = imuKey(device);
        opt.textContent = imuLabel(device);
        opt.dataset.name = device.name || 'WT IMU';
        opt.dataset.address = device.address || '';
        opt.dataset.transport = device.transport || 'ble';
        opt.dataset.adapterPort = device.adapter_port || '';
        opt.dataset.adapterDeviceIndex = device.adapter_device_index || '';
        opt.dataset.adapterPassive = device.adapter_passive ? '1' : '';
        select.appendChild(opt);
      }
      if ([...select.options].some(option => option.value === selected)) select.value = selected;
    }

    async function loadKnownImus() {
      mergeImuDevices(await api('/api/imu/known'));
      for (const slot of imuSlots) {
        const previous = $(slot + '_device').value;
        const fallbackDevice = defaultImuForSlot(slot);
        const fallback = fallbackDevice ? imuKey(fallbackDevice) : '';
        populateImuSelect(slot, previous || fallback);
      }
    }

    async function connectSelectedImu(slot) {
      const select = $(slot + '_device');
      const address = select.value;
      const option = select.selectedOptions[0];
      if (!address || !option) {
        throw new Error(`No IMU selected for ${slot}`);
      }
      const name = option.dataset?.name || option.textContent || 'WT IMU';
      const payload = {
        slot,
        name,
        address: option.dataset?.address || '',
        transport: option.dataset?.transport || 'ble',
        adapter_port: option.dataset?.adapterPort || '',
        adapter_device_index: option.dataset?.adapterDeviceIndex || '',
        adapter_passive: option.dataset?.adapterPassive === '1',
      };
      updateImuPanel(slot, {connection_state: 'connecting', selected_name: name, selected_address: payload.address});
      await api('/api/imu/select', {method:'POST', body: JSON.stringify(payload)});
      await refreshStatus();
    }

    async function disconnectImu(slot) {
      await api('/api/imu/stop', {method:'POST', body: JSON.stringify({slot})});
      await refreshStatus();
    }

    async function connectAllImus() {
      for (const slot of imuSlots) {
        if (!$(slot + '_device').value) continue;
        await connectSelectedImu(slot);
        await new Promise(resolve => setTimeout(resolve, 6500));
      }
      await refreshStatus();
    }

    async function disconnectAllImus() {
      for (const slot of imuSlots) await disconnectImu(slot);
    }

    function formatVec(values, digits = 3) {
      if (!Array.isArray(values)) return '--';
      return values.map(value => Number(value).toFixed(digits)).join(', ');
    }

    function formatUnixNs(timestampNs) {
      const value = Number(timestampNs);
      if (!Number.isFinite(value) || value <= 0) return '--';
      return new Date(value / 1e6).toISOString();
    }

    function metric(label, value) {
      return `<div class="metric"><strong>${label}</strong><span>${escapeHtml(value)}</span></div>`;
    }

    function updateImuPanel(slot, status = {}) {
      const sample = status.last_sample || {};
      const state = status.connection_state || 'idle';
      const name = status.selected_name || '未选择';
      const target = status.adapter_port ? `${status.adapter_port} -> ${status.selected_address || ''}` : (status.selected_address || '');
      const age = status.last_sample_age_s == null ? '无数据' : `${status.last_sample_age_s}s 前`;
      const recording = status.recording ? 'recording' : 'not recording';
      const count = status.sample_count ?? 0;
      const error = status.last_error ? `<div class="status" style="color: var(--bad);">${escapeHtml(status.last_error.split('\n')[0])}</div>` : '';
      const unixNs = sample.timestamp_unix_ns ? String(sample.timestamp_unix_ns) : '--';
      const monotonicNs = sample.timestamp_monotonic_ns ? String(sample.timestamp_monotonic_ns) : '--';
      $(slot + '_status').innerHTML = `
        <div>${escapeHtml(name)} | ${escapeHtml(state)} | ${escapeHtml(target)} | samples ${count} | latest ${escapeHtml(age)} | ${escapeHtml(recording)}</div>
        <div class="imu-data">
          ${metric('accel_mps2 XYZ', formatVec(sample.accel_mps2))}
          ${metric('gyro_radps XYZ', formatVec(sample.gyro_radps))}
          ${metric('euler_deg RPY', formatVec(sample.euler_deg))}
          ${metric('quat_wxyz', formatVec(sample.quat_wxyz))}
          ${metric('mag XYZ', formatVec(sample.mag))}
          ${metric('unix time', formatUnixNs(sample.timestamp_unix_ns))}
          ${metric('timestamp_unix_ns', unixNs)}
          ${metric('timestamp_monotonic_ns', monotonicNs)}
        </div>
        ${error}`;
    }

    async function refreshStatus() {
      const data = await api('/api/status');
      const record = data.recording?.message || 'idle';
      const camera = data.camera?.message || 'idle';
      const visualization = data.visualization?.message || 'idle';
      recordingActive = Boolean(data.recording?.active);
      visualizationActive = Boolean(data.visualization?.active);
      updateRecordButton();
      updateVisualizeButton();
      for (const slot of imuSlots) updateImuPanel(slot, data.imus?.[slot]);
      const cameraError = data.camera?.error ? ` <span style="color: var(--bad);">${escapeHtml(data.camera.error.split('\n')[0])}</span>` : '';
      const visualizationError = data.visualization?.error ? ` <span style="color: var(--bad);">${escapeHtml(data.visualization.error.split('\n')[0])}</span>` : '';
      const summary = data.session_summary ? sessionSummaryText(data.session_summary) : '';
      $('status').innerHTML = `Record: <b>${record}</b> Camera: <b>${camera}</b>${cameraError} RViz: <b>${visualization}</b>${visualizationError}${summary}`;
      $('log').textContent = JSON.stringify(data, null, 2);
    }

    function sessionSummaryText(summary) {
      const ok = Boolean(summary.ok_for_camera_replay);
      const color = ok ? 'var(--good)' : 'var(--bad)';
      const frames = summary.camera_frame_count ?? 0;
      const imus = summary.imu_counts || {};
      const imuText = Object.entries(imus).map(([name, count]) => `${name}:${count}`).join(' ');
      const warn = summary.has_capture_warnings ? ' warnings' : '';
      return ` <span style="color:${color};">Session frames:${frames}${warn}${imuText ? ' IMU ' + escapeHtml(imuText) : ''}</span>`;
    }

    function updateRecordButton() {
      const button = $('recordToggle');
      button.textContent = recordingActive ? '停止 Record' : '开始 Record';
      button.classList.toggle('primary', !recordingActive);
      button.classList.toggle('stop', recordingActive);
    }

    function updateVisualizeButton() {
      const button = $('visualizeToggle');
      button.textContent = visualizationActive ? '停止 RViz 可视化' : '启动 RViz 可视化';
      button.classList.toggle('primary', !visualizationActive);
      button.classList.toggle('stop', visualizationActive);
    }

    async function toggleRecord() {
      if (recordingActive) {
        await api('/api/record/stop', {method:'POST', body: '{}'});
        stopRecordingPreviews();
        restartCameraPreviews();
      } else {
        clearCameraPreviews();
        await api('/api/preview/stop', {method:'POST', body: '{}'});
        await new Promise(resolve => setTimeout(resolve, 1000));
        await api('/api/record/start', {method:'POST', body: JSON.stringify(recordPayload())});
        startRecordingPreviews();
      }
      await refreshStatus();
    }

    async function toggleVisualization() {
      if (visualizationActive) {
        await api('/api/visualization/stop', {method:'POST', body: '{}'});
      } else {
        await api('/api/visualization/start', {method:'POST', body: JSON.stringify(recordPayload())});
      }
      await refreshStatus();
    }

    $('refreshCameras').onclick = () => runAction(() => loadCameras(false));
    $('assignCameras').onclick = () => runAction(() => loadCameras(true));
    $('recordToggle').onclick = () => runAction(toggleRecord);
    $('visualizeToggle').onclick = () => runAction(toggleVisualization);
    cameraLayout.onchange = applyCameraLayout;
    $('scanImu').onclick = () => runAction(async () => {
      mergeImuDevices(await api('/api/imu/scan?timeout_s=20'));
      for (const slot of imuSlots) {
        const previous = $(slot + '_device').value;
        const fallbackDevice = defaultImuForSlot(slot);
        const fallback = fallbackDevice ? imuKey(fallbackDevice) : '';
        populateImuSelect(slot, previous || fallback);
      }
      $('log').textContent = JSON.stringify(imuDevices, null, 2);
    });
    for (const slot of imuSlots) {
      populateImuSelect(slot);
      $(slot + '_connect').onclick = () => runAction(() => connectSelectedImu(slot));
      $(slot + '_disconnect').onclick = () => runAction(() => disconnectImu(slot));
    }
    $('connectAllImu').onclick = () => runAction(connectAllImus);
    $('disconnectAllImu').onclick = () => runAction(disconnectAllImus);

    runAction(async () => {
      applyCameraLayout();
      await loadProfile();
      await loadKnownImus();
      await loadCameras(true);
      await refreshStatus();
    });
    setInterval(() => runAction(refreshStatus), 1000);
  </script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local 3D Motion capture dashboard.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--head-imu-adapter-port", default=None, help="Override the default head_imu adapter port.")
    parser.add_argument("--wrist-imu-adapter-port", default=None, help="Override the default wrist_imu adapter port.")
    args = parser.parse_args()

    _apply_serial_adapter_defaults(
        {
            "head_imu": args.head_imu_adapter_port,
            "wrist_imu": args.wrist_imu_adapter_port,
        }
    )

    server = DashboardServer((args.host, args.port), CaptureJobs())
    url = f"http://{args.host}:{args.port}/"
    print(f"3D Motion capture dashboard: {url}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard...")
    finally:
        server.jobs.shutdown()
        server.stop_previews()
        server.server_close()


if __name__ == "__main__":
    main()
