# capture_dashboard

本地浏览器采集控制台，用来把常用 CLI 参数变成可视化表单。

启动：

```bash
source .venv/bin/activate
python scripts/capture_dashboard.py
```

默认地址：

```text
http://127.0.0.1:8766/
```

当前能力：

- 列出 V4L2 camera。
- 为 `C0-C3` 选择 `/dev/video*`。
- 预览每路 camera 画面。
- 设置 `fps`、`duration_s`、`width`、`height`、`output_dir`。
- 后台启动/停止四目采集。
- 扫描 WT BLE IMU。
- 设置 IMU address、sensor ID、duration、output。
- 后台启动/停止 IMU JSONL 采集。

注意：

- 预览和采集可能同时占用同一个 camera。如果设备不支持多进程打开，采集前先停止或刷新预览。
- Dashboard 只是采集层工具，不做 AprilTag / OpenVINS / ESKF 处理。
- 离线处理仍使用 `scripts/process_apriltag_session.py`。

Recording preview note:

- Idle preview uses direct `/stream` camera access.
- During Record, the UI previews the latest JPEG already written under the
  active session's `cameras/C*/` directory.
- This avoids opening the same V4L2 camera twice and makes the preview reflect
  what is actually being saved.
