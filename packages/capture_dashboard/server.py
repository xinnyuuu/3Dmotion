from __future__ import annotations

import argparse
import asyncio
import json
import socketserver
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from packages.imu_ble_bridge.wt901 import WT901BleClient, scan_wt_devices, write_jsonl_sample
from packages.quad_camera_capture.capture import CameraSource, QuadCameraCapture
from packages.quad_camera_capture.v4l2 import find_capture_devices, get_camera_config
from packages.session_tools.validate_session import validate_session


DEFAULT_CAMERA_WIDTH = 1280
DEFAULT_CAMERA_HEIGHT = 720
DEFAULT_CAMERA_FPS = 15.0
DEFAULT_PREVIEW_FPS = 8.0


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
    connection_state: str = "idle"
    last_sample_monotonic_ns: int | None = None
    last_error: str | None = None
    recording: bool = False
    output: str | None = None
    last_sample: dict | None = None

    def as_dict(self) -> dict:
        data = asdict(self)
        if self.last_sample_monotonic_ns is not None:
            age_s = (time.monotonic_ns() - self.last_sample_monotonic_ns) / 1_000_000_000
            data["last_sample_age_s"] = round(age_s, 2)
        else:
            data["last_sample_age_s"] = None
        return data


class CaptureJobs:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.camera = JobStatus(kind="camera")
        self.recording = JobStatus(kind="record")
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
        self._session_dir: Path | None = None
        self._camera_thread: threading.Thread | None = None
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
        if not address:
            raise RuntimeError("Missing BLE address.")

        self._imu_stop_events[slot].set()
        with self.lock:
            self._imu_stop_events[slot] = threading.Event()
            self._imu_record_paths[slot] = None
            self.imu_slots[slot] = ImuSlotStatus(
                slot=slot,
                selected_name=name,
                selected_address=address,
                connection_state="connecting",
            )
            stop_event = self._imu_stop_events[slot]
        thread = threading.Thread(target=self._run_imu_connection, args=(slot, address, stop_event), daemon=True)
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
            return {
                "recording": self.recording.as_dict(),
                "camera": self.camera.as_dict(),
                "imus": {slot: status.as_dict() for slot, status in self.imu_slots.items()},
                "session_summary": self.session_summary,
            }

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

    def _run_imu_connection(self, slot: str, address: str, stop_event: threading.Event) -> None:
        try:
            sensor_id = slot

            async def run_client() -> None:
                client = WT901BleClient(
                    address,
                    sensor_id,
                    lambda sample: self._handle_imu_sample(slot, sample),
                    on_connected=lambda: self._mark_imu_connected(slot),
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

            asyncio.run(run_client())
            with self.lock:
                if self.imu_slots[slot].selected_address == address:
                    self.imu_slots[slot].connection_state = "disconnected"
                    self.imu_slots[slot].recording = False
                    self._imu_record_paths[slot] = None
        except Exception as exc:
            with self.lock:
                if self.imu_slots[slot].selected_address == address:
                    self.imu_slots[slot].connection_state = "failed"
                    self.imu_slots[slot].recording = False
                    self.imu_slots[slot].last_error = f"{exc}\n{traceback.format_exc()}"
                    self._imu_record_paths[slot] = None

    def _mark_imu_connected(self, slot: str) -> None:
        with self.lock:
            self.imu_slots[slot].connection_state = "connected"
            self.imu_slots[slot].last_error = None

    def _handle_imu_sample(self, slot: str, sample) -> None:
        path = None
        with self.lock:
            self.imu_slots[slot].last_sample_monotonic_ns = sample.timestamp_monotonic_ns
            self.imu_slots[slot].last_sample = asdict(sample)
            path = self._imu_record_paths.get(slot)
        if path is not None:
            write_jsonl_sample(path, sample)


class DashboardServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], jobs: CaptureJobs) -> None:
        super().__init__(address, DashboardHandler)
        self.jobs = jobs


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
                    "preview_fps": DEFAULT_PREVIEW_FPS,
                }
            )
            return
        if parsed.path == "/api/imu/scan":
            timeout_s = _optional_float(parse_qs(parsed.query).get("timeout_s", ["6"])[0]) or 6.0
            devices = [{"name": name, "address": address} for name, address in asyncio.run(scan_wt_devices(timeout_s))]
            self._send_json(devices)
            return
        if parsed.path == "/stream":
            params = parse_qs(parsed.query)
            source = unquote(params.get("source", params.get("device", [""]))[0])
            fourcc = unquote(params.get("format", [""])[0])
            width = _optional_int(params.get("width", [None])[0])
            height = _optional_int(params.get("height", [None])[0])
            fps = _optional_float(params.get("fps", [None])[0]) or DEFAULT_PREVIEW_FPS
            self._stream_camera(source, fourcc, width, height, fps)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        payload = self._read_json()
        try:
            if parsed.path == "/api/camera/start":
                self._send_json(self.server.jobs.start_record(payload).as_dict())
                return
            if parsed.path == "/api/camera/stop":
                self._send_json(self.server.jobs.stop_record().as_dict())
                return
            if parsed.path == "/api/record/start":
                self._send_json(self.server.jobs.start_record(payload).as_dict())
                return
            if parsed.path == "/api/record/stop":
                self._send_json(self.server.jobs.stop_record().as_dict())
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
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def _stream_camera(self, source: str, fourcc: str, width: int | None, height: int | None, fps: float) -> None:
        if not source:
            self.send_error(HTTPStatus.BAD_REQUEST, "Missing source")
            return
        try:
            import cv2
        except ImportError:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "opencv-python is not installed")
            return
        cap = cv2.VideoCapture(_parse_source(source), cv2.CAP_V4L2)
        if fourcc and len(fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            cap.set(cv2.CAP_PROP_FPS, fps)
        if not cap.isOpened():
            cap.release()
            self.send_error(HTTPStatus.BAD_REQUEST, f"Could not open camera: {source}")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        delay = 1.0 / max(1.0, min(fps, 15.0))
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(delay)
                    continue
                encode_ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if not encode_ok:
                    continue
                body = encoded.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(body)}\r\n\r\n".encode("ascii"))
                self.wfile.write(body)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(delay)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return
        finally:
            cap.release()

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
    try:
        return int(source)
    except (TypeError, ValueError):
        return source


def _optional_int(value) -> int | None:
    if value in (None, ""):
        return None
    return int(value)


def _optional_float(value) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _camera_sources_from_payload(payload: dict) -> list[CameraSource]:
    sources = []
    camera_configs = payload.get("camera_configs") or {}
    for index in range(4):
        camera_id = f"C{index}"
        raw_source = str(payload.get(camera_id, "")).strip()
        if raw_source:
            config = camera_configs.get(camera_id) or {}
            fourcc = str(config.get("format") or "").strip() or None
            sources.append(CameraSource(camera_id=camera_id, source=_parse_source(raw_source), fourcc=fourcc))
    return sources


def _imu_slot(payload: dict) -> str:
    slot = str(payload.get("slot") or "wrist_imu").strip()
    if slot not in {"head_imu", "wrist_imu"}:
        raise RuntimeError(f"Unknown IMU slot: {slot}")
    return slot


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
    main { display: grid; grid-template-columns: minmax(360px, 430px) minmax(0, 1fr); gap: 14px; padding: 14px; }
    section { background: var(--panel); border: 1px solid var(--line); padding: 12px; }
    h2 { margin: 0 0 10px; font-size: 15px; }
    label { display: grid; gap: 5px; color: var(--muted); font-size: 12px; margin-bottom: 9px; }
    input, select, button { width: 100%; border: 1px solid var(--line); background: #0f131b; color: var(--text); padding: 8px; font: inherit; }
    button { cursor: pointer; background: #1d2430; }
    button.primary { background: var(--accent); border-color: var(--accent); font-weight: 700; }
    button.stop { background: var(--bad); border-color: var(--bad); font-weight: 700; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    .camera-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
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
  </style>
</head>
<body>
  <header>
    <h1>3D Motion Capture Dashboard</h1>
    <div class="status" id="status">加载中</div>
  </header>
  <main>
    <div>
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
        <h2>Head IMU</h2>
        <label>Device <select id="head_imu_device"></select></label>
        <div class="status" id="head_imu_status">未选择</div>
        <h2 style="margin-top: 14px;">Wrist IMU</h2>
        <label>Device <select id="wrist_imu_device"></select></label>
        <div class="status" id="wrist_imu_status">未选择</div>
      </section>
      <section style="margin-top: 14px;">
        <h2>状态</h2>
        <pre id="log"></pre>
      </section>
    </div>
    <section>
      <h2>Camera 预览</h2>
      <div class="camera-grid" id="cameraGrid"></div>
    </section>
  </main>
  <script>
    const SLOT_COUNT = 4;
    const cameraIds = ['C0', 'C1', 'C2', 'C3'];
    const imuSlots = ['head_imu', 'wrist_imu'];
    const PREFERRED_FPS = 15;
    const $ = id => document.getElementById(id);
    const cameraGrid = $('cameraGrid');
    let cameraProfile = {width: 1280, height: 720, fps: 15, preview_fps: 8};
    let cameras = [];
    let configs = {};
    let selections = Array.from({length: SLOT_COUNT}, () => '');
    let streamConfigs = Array.from({length: SLOT_COUNT}, () => ({format: '', size: '', fps: ''}));
    let rotations = Array.from({length: SLOT_COUNT}, () => 0);
    let imuDevices = [];
    let recordingActive = false;
    let recordingPreviewTimer = null;

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
      return formats.find(format => format.format === current)?.format || formats[0]?.format || '';
    }

    function selectedSize(path, slot, formatName) {
      const format = (configs[path] || []).find(item => item.format === formatName);
      const current = streamConfigs[slot]?.size;
      return format?.sizes?.find(size => (size.label || `${size.width}x${size.height}`) === current)?.label ||
        (format?.sizes?.[0] ? (format.sizes[0].label || `${format.sizes[0].width}x${format.sizes[0].height}`) : '');
    }

    function selectedFps(path, slot, formatName, sizeLabel) {
      const format = (configs[path] || []).find(item => item.format === formatName);
      const size = format?.sizes?.find(item => (item.label || `${item.width}x${item.height}`) === sizeLabel);
      const current = Number(streamConfigs[slot]?.fps || 0);
      if (size?.fps?.includes(current)) return String(current);
      const preferred = size?.fps?.find(value => value <= PREFERRED_FPS) || size?.fps?.[size.fps.length - 1];
      return String(preferred || '');
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
      const format = (configs[path] || []).find(item => item.format === formatName);
      const size = format?.sizes?.find(item => (item.label || `${item.width}x${item.height}`) === sizeLabel);
      return (size?.fps || []).map(fps => {
        const selected = String(fps) === String(selectedFpsValue) ? ' selected' : '';
        return `<option value="${fps}"${selected}>${fps} fps</option>`;
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
      streamConfigs[slot] = {format, size, fps};
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
      if (config.fps) params.set('fps', Math.min(Number(config.fps), Number(cameraProfile.preview_fps)));
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
          img.src = streamUrl(selections[i], cfg);
          overlay.textContent = `${cameraLabel(selections[i])}${cfg.size ? ` | ${cfg.format} ${cfg.size} @ ${cfg.fps}fps` : ''}`;
          badge.textContent = selections[i] ? 'LIVE' : 'IDLE';
          badge.className = `badge ${selections[i] ? 'live' : 'idle'}`;
          applyRotation(img, rotations[i]);
        };
        cameraSelect.addEventListener('change', () => {
          selections[i] = cameraSelect.value;
          streamConfigs[i] = {format: '', size: '', fps: ''};
          updateControls();
          restartStream();
        });
        formatSelect.addEventListener('change', () => {
          streamConfigs[i].format = formatSelect.value;
          streamConfigs[i].size = '';
          streamConfigs[i].fps = '';
          updateControls();
          restartStream();
        });
        sizeSelect.addEventListener('change', () => {
          streamConfigs[i].size = sizeSelect.value;
          streamConfigs[i].fps = '';
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
      const data = {session_root: $('session_root').value, width: cfg.width, height: cfg.height, fps: cfg.fps};
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
      return `${device.name || 'WT IMU'} ${device.address}`;
    }

    function populateImuSelect(slot, selected = '') {
      const select = $(slot + '_device');
      select.innerHTML = '<option value="">No IMU</option>';
      for (const device of imuDevices) {
        const opt = document.createElement('option');
        opt.value = device.address;
        opt.textContent = imuLabel(device);
        opt.dataset.name = device.name || 'WT IMU';
        select.appendChild(opt);
      }
      if ([...select.options].some(option => option.value === selected)) select.value = selected;
    }

    async function selectImu(slot) {
      const select = $(slot + '_device');
      const address = select.value;
      if (!address) {
        await api('/api/imu/stop', {method:'POST', body: JSON.stringify({slot})});
        await refreshStatus();
        return;
      }
      const name = select.selectedOptions[0]?.dataset?.name || select.selectedOptions[0]?.textContent || 'WT IMU';
      updateImuPanel(slot, {connection_state: 'connecting', selected_name: name, selected_address: address});
      await api('/api/imu/select', {method:'POST', body: JSON.stringify({slot, name, address})});
      await refreshStatus();
    }

    function formatVec(values, digits = 3) {
      if (!Array.isArray(values)) return '--';
      return values.map(value => Number(value).toFixed(digits)).join(', ');
    }

    function metric(label, value) {
      return `<div class="metric"><strong>${label}</strong><span>${escapeHtml(value)}</span></div>`;
    }

    function updateImuPanel(slot, status = {}) {
      const sample = status.last_sample || {};
      const state = status.connection_state || 'idle';
      const name = status.selected_name || '未选择';
      const age = status.last_sample_age_s == null ? '无数据' : `${status.last_sample_age_s}s 前`;
      const recording = status.recording ? 'recording' : 'not recording';
      const error = status.last_error ? `<div class="status" style="color: var(--bad);">${escapeHtml(status.last_error.split('\n')[0])}</div>` : '';
      $(slot + '_status').innerHTML = `
        <div>${escapeHtml(name)} | ${escapeHtml(state)} | sample ${escapeHtml(age)} | ${escapeHtml(recording)}</div>
        <div class="imu-data">
          ${metric('accel_mps2 XYZ', formatVec(sample.accel_mps2))}
          ${metric('gyro_radps XYZ', formatVec(sample.gyro_radps))}
          ${metric('euler_deg RPY', formatVec(sample.euler_deg))}
          ${metric('quat_wxyz', formatVec(sample.quat_wxyz))}
          ${metric('mag XYZ', formatVec(sample.mag))}
          ${metric('timestamp', sample.timestamp_unix_ns ? String(sample.timestamp_unix_ns) : '--')}
        </div>
        ${error}`;
    }

    async function refreshStatus() {
      const data = await api('/api/status');
      const record = data.recording?.message || 'idle';
      const camera = data.camera?.message || 'idle';
      recordingActive = Boolean(data.recording?.active);
      updateRecordButton();
      for (const slot of imuSlots) updateImuPanel(slot, data.imus?.[slot]);
      const cameraError = data.camera?.error ? ` <span style="color: var(--bad);">${escapeHtml(data.camera.error.split('\n')[0])}</span>` : '';
      const summary = data.session_summary ? sessionSummaryText(data.session_summary) : '';
      $('status').innerHTML = `Record: <b>${record}</b> Camera: <b>${camera}</b>${cameraError}${summary}`;
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

    async function toggleRecord() {
      if (recordingActive) {
        await api('/api/record/stop', {method:'POST', body: '{}'});
        stopRecordingPreviews();
        restartCameraPreviews();
      } else {
        clearCameraPreviews();
        await new Promise(resolve => setTimeout(resolve, 1000));
        await api('/api/record/start', {method:'POST', body: JSON.stringify(recordPayload())});
        startRecordingPreviews();
      }
      await refreshStatus();
    }

    $('refreshCameras').onclick = () => runAction(() => loadCameras(false));
    $('assignCameras').onclick = () => runAction(() => loadCameras(true));
    $('recordToggle').onclick = () => runAction(toggleRecord);
    $('scanImu').onclick = () => runAction(async () => {
      imuDevices = await api('/api/imu/scan?timeout_s=6');
      for (const slot of imuSlots) {
        const previous = $(slot + '_device').value;
        populateImuSelect(slot, previous);
      }
      if (imuDevices[0] && !$('head_imu_device').value) $('head_imu_device').value = imuDevices[0].address;
      if (imuDevices[1] && !$('wrist_imu_device').value) $('wrist_imu_device').value = imuDevices[1].address;
      for (const slot of imuSlots) {
        if ($(slot + '_device').value) await selectImu(slot);
      }
      $('log').textContent = JSON.stringify(imuDevices, null, 2);
    });
    for (const slot of imuSlots) {
      populateImuSelect(slot);
      $(slot + '_device').onchange = () => runAction(() => selectImu(slot));
    }

    runAction(async () => {
      await loadProfile();
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
    args = parser.parse_args()

    server = DashboardServer((args.host, args.port), CaptureJobs())
    url = f"http://{args.host}:{args.port}/"
    print(f"3D Motion capture dashboard: {url}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
