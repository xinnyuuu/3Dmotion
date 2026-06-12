from __future__ import annotations

import argparse
import asyncio
import json
import socketserver
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from packages.imu_ble_bridge.wt901 import WT901BleClient, scan_wt_devices, write_jsonl_sample
from packages.quad_camera_capture.capture import CameraSource, QuadCameraCapture
from packages.quad_camera_capture.v4l2 import find_capture_devices, get_camera_config


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


class CaptureJobs:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.camera = JobStatus(kind="camera")
        self.imu = JobStatus(kind="imu")
        self._camera_stop = threading.Event()
        self._imu_stop = threading.Event()

    def start_camera(self, payload: dict) -> JobStatus:
        with self.lock:
            if self.camera.active:
                raise RuntimeError("Camera capture is already running.")
            self._camera_stop = threading.Event()
            self.camera = JobStatus(kind="camera", active=True, started_at=time.monotonic(), message="starting")
            thread = threading.Thread(target=self._run_camera, args=(payload, self._camera_stop), daemon=True)
            thread.start()
            return self.camera

    def stop_camera(self) -> JobStatus:
        self._camera_stop.set()
        with self.lock:
            if self.camera.active:
                self.camera.message = "stopping"
            return self.camera

    def start_imu(self, payload: dict) -> JobStatus:
        with self.lock:
            if self.imu.active:
                raise RuntimeError("IMU capture is already running.")
            self._imu_stop = threading.Event()
            self.imu = JobStatus(kind="imu", active=True, started_at=time.monotonic(), message="starting")
            thread = threading.Thread(target=self._run_imu, args=(payload, self._imu_stop), daemon=True)
            thread.start()
            return self.imu

    def stop_imu(self) -> JobStatus:
        self._imu_stop.set()
        with self.lock:
            if self.imu.active:
                self.imu.message = "stopping after current BLE loop"
            return self.imu

    def snapshot(self) -> dict:
        with self.lock:
            return {"camera": self.camera.as_dict(), "imu": self.imu.as_dict()}

    def _run_camera(self, payload: dict, stop_event: threading.Event) -> None:
        try:
            sources = []
            for index in range(4):
                camera_id = f"C{index}"
                source = str(payload.get(camera_id, "")).strip()
                if not source:
                    raise RuntimeError(f"Missing source for {camera_id}.")
                sources.append(CameraSource(camera_id=camera_id, source=_parse_source(source)))
            output_dir = Path(str(payload.get("output_dir") or "data/raw/quad_camera_dashboard"))
            fps = float(payload.get("fps") or 15)
            duration_s = _optional_float(payload.get("duration_s"))
            width = _optional_int(payload.get("width"))
            height = _optional_int(payload.get("height"))
            with self.lock:
                self.camera.message = "capturing"
                self.camera.output = str(output_dir)
            QuadCameraCapture(sources=sources, output_dir=output_dir, target_fps=fps, width=width, height=height).run(
                duration_s=duration_s,
                stop_event=stop_event,
            )
            with self.lock:
                self.camera.active = False
                self.camera.finished_at = time.monotonic()
                self.camera.message = "finished"
        except Exception as exc:
            with self.lock:
                self.camera.active = False
                self.camera.finished_at = time.monotonic()
                self.camera.message = "failed"
                self.camera.error = f"{exc}\n{traceback.format_exc()}"

    def _run_imu(self, payload: dict, stop_event: threading.Event) -> None:
        try:
            address = str(payload.get("address") or "").strip()
            if not address:
                raise RuntimeError("Missing BLE address.")
            sensor_id = str(payload.get("sensor_id") or "wrist_imu")
            output = Path(str(payload.get("output") or "data/raw/wrist_imu.jsonl"))
            duration_s = _optional_float(payload.get("duration_s"))
            output.parent.mkdir(parents=True, exist_ok=True)
            with self.lock:
                self.imu.message = "capturing"
                self.imu.output = str(output)

            async def run_client() -> None:
                client = WT901BleClient(address, sensor_id, lambda sample: write_jsonl_sample(output, sample))
                start = time.monotonic()
                task = asyncio.create_task(client.run(duration_s=duration_s))
                while not task.done():
                    if stop_event.is_set():
                        task.cancel()
                        break
                    if duration_s is not None and time.monotonic() - start >= duration_s + 1.0:
                        break
                    await asyncio.sleep(0.1)
                try:
                    await task
                except asyncio.CancelledError:
                    return

            asyncio.run(run_client())
            with self.lock:
                self.imu.active = False
                self.imu.finished_at = time.monotonic()
                self.imu.message = "finished"
        except Exception as exc:
            with self.lock:
                self.imu.active = False
                self.imu.finished_at = time.monotonic()
                self.imu.message = "failed"
                self.imu.error = f"{exc}\n{traceback.format_exc()}"


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
        if parsed.path == "/api/imu/scan":
            timeout_s = _optional_float(parse_qs(parsed.query).get("timeout_s", ["6"])[0]) or 6.0
            devices = [{"name": name, "address": address} for name, address in asyncio.run(scan_wt_devices(timeout_s))]
            self._send_json(devices)
            return
        if parsed.path == "/stream":
            params = parse_qs(parsed.query)
            source = unquote(params.get("source", [""])[0])
            width = _optional_int(params.get("width", [None])[0])
            height = _optional_int(params.get("height", [None])[0])
            fps = _optional_float(params.get("fps", ["10"])[0]) or 10.0
            self._stream_camera(source, width, height, fps)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        payload = self._read_json()
        try:
            if parsed.path == "/api/camera/start":
                self._send_json(self.server.jobs.start_camera(payload).as_dict())
                return
            if parsed.path == "/api/camera/stop":
                self._send_json(self.server.jobs.stop_camera().as_dict())
                return
            if parsed.path == "/api/imu/start":
                self._send_json(self.server.jobs.start_imu(payload).as_dict())
                return
            if parsed.path == "/api/imu/stop":
                self._send_json(self.server.jobs.stop_imu().as_dict())
                return
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def _stream_camera(self, source: str, width: int | None, height: int | None, fps: float) -> None:
        if not source:
            self.send_error(HTTPStatus.BAD_REQUEST, "Missing source")
            return
        try:
            import cv2
        except ImportError:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "opencv-python is not installed")
            return
        cap = cv2.VideoCapture(_parse_source(source))
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
    .view { min-height: 260px; border: 1px solid var(--line); background: #05070b; display: grid; grid-template-rows: auto minmax(0, 1fr); }
    .view strong { padding: 8px; border-bottom: 1px solid var(--line); font-size: 12px; color: var(--muted); }
    .view img { width: 100%; height: 100%; object-fit: contain; min-height: 220px; }
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
          <button id="preview">更新预览</button>
        </div>
        <label>C0 <select id="C0"></select></label>
        <label>C1 <select id="C1"></select></label>
        <label>C2 <select id="C2"></select></label>
        <label>C3 <select id="C3"></select></label>
        <div class="row">
          <label>FPS <input id="fps" value="15"></label>
          <label>Duration s <input id="duration_s" value="5"></label>
        </div>
        <div class="row">
          <label>Width <input id="width" value="1920"></label>
          <label>Height <input id="height" value="1080"></label>
        </div>
        <label>Output dir <input id="output_dir" value="data/raw/quad_camera_test"></label>
        <div class="actions">
          <button class="primary" id="startCamera">开始相机采集</button>
          <button class="stop" id="stopCamera">停止相机采集</button>
        </div>
      </section>
      <section style="margin-top: 14px;">
        <h2>IMU 设置</h2>
        <div class="row">
          <button id="scanImu">扫描 IMU</button>
          <select id="imuDevices"></select>
        </div>
        <label>BLE address <input id="imu_address"></label>
        <label>Sensor ID <input id="sensor_id" value="wrist_imu"></label>
        <div class="row">
          <label>Duration s <input id="imu_duration_s" value="10"></label>
          <label>Output <input id="imu_output" value="data/raw/wrist_imu.jsonl"></label>
        </div>
        <div class="actions">
          <button class="primary" id="startImu">开始 IMU 采集</button>
          <button class="stop" id="stopImu">停止 IMU 采集</button>
        </div>
      </section>
      <section style="margin-top: 14px;">
        <h2>状态</h2>
        <pre id="log"></pre>
      </section>
    </div>
    <section>
      <h2>Camera 预览</h2>
      <div class="grid">
        <div class="view"><strong>C0</strong><img id="preview_C0"></div>
        <div class="view"><strong>C1</strong><img id="preview_C1"></div>
        <div class="view"><strong>C2</strong><img id="preview_C2"></div>
        <div class="view"><strong>C3</strong><img id="preview_C3"></div>
      </div>
    </section>
  </main>
  <script>
    const ids = ['C0', 'C1', 'C2', 'C3'];
    const $ = id => document.getElementById(id);

    async function api(path, options = {}) {
      const res = await fetch(path, options);
      const text = await res.text();
      const data = text ? JSON.parse(text) : {};
      if (!res.ok) throw new Error(data.error || res.statusText);
      return data;
    }

    function payload() {
      return {
        C0: $('C0').value, C1: $('C1').value, C2: $('C2').value, C3: $('C3').value,
        fps: $('fps').value, duration_s: $('duration_s').value,
        width: $('width').value, height: $('height').value, output_dir: $('output_dir').value
      };
    }

    async function loadCameras() {
      const devices = await api('/api/cameras?configs=1');
      for (const id of ids) {
        const select = $(id);
        const previous = select.value;
        select.innerHTML = '';
        for (const device of devices) {
          const opt = document.createElement('option');
          opt.value = device.path;
          opt.textContent = `${device.path} ${device.name || ''}`;
          select.appendChild(opt);
        }
        if ([...select.options].some(o => o.value === previous)) select.value = previous;
      }
      $('log').textContent = JSON.stringify(devices, null, 2);
    }

    function updatePreview() {
      const p = payload();
      for (const id of ids) {
        const img = $('preview_' + id);
        img.src = `/stream?source=${encodeURIComponent(p[id])}&width=${encodeURIComponent(p.width)}&height=${encodeURIComponent(p.height)}&fps=8&v=${Date.now()}`;
      }
    }

    async function refreshStatus() {
      const data = await api('/api/status');
      $('status').innerHTML = `Camera: <b>${data.camera.message}</b> IMU: <b>${data.imu.message}</b>`;
      $('log').textContent = JSON.stringify(data, null, 2);
    }

    $('refreshCameras').onclick = loadCameras;
    $('preview').onclick = updatePreview;
    $('startCamera').onclick = async () => { await api('/api/camera/start', {method:'POST', body: JSON.stringify(payload())}); await refreshStatus(); };
    $('stopCamera').onclick = async () => { await api('/api/camera/stop', {method:'POST', body: '{}'}); await refreshStatus(); };
    $('scanImu').onclick = async () => {
      const devices = await api('/api/imu/scan?timeout_s=6');
      $('imuDevices').innerHTML = '';
      for (const device of devices) {
        const opt = document.createElement('option');
        opt.value = device.address;
        opt.textContent = `${device.name} ${device.address}`;
        $('imuDevices').appendChild(opt);
      }
      if (devices[0]) $('imu_address').value = devices[0].address;
      $('log').textContent = JSON.stringify(devices, null, 2);
    };
    $('imuDevices').onchange = () => { $('imu_address').value = $('imuDevices').value; };
    $('startImu').onclick = async () => {
      await api('/api/imu/start', {method:'POST', body: JSON.stringify({
        address: $('imu_address').value, sensor_id: $('sensor_id').value,
        duration_s: $('imu_duration_s').value, output: $('imu_output').value
      })});
      await refreshStatus();
    };
    $('stopImu').onclick = async () => { await api('/api/imu/stop', {method:'POST', body: '{}'}); await refreshStatus(); };

    loadCameras().then(updatePreview).then(refreshStatus);
    setInterval(refreshStatus, 1500);
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
