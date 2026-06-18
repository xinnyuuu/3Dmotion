# capture_dashboard

本地浏览器采集控制台。完整启动和采集流程见 `../../docs/system_workflow.md`。

## 职责

- 列出 V4L2 camera，并为 `C0-C3` 选择设备和 format。
- 预览每路 camera 画面。
- 扫描 WT BLE IMU，并分别绑定到 `head_imu` / `wrist_imu`。
- 用一个 Record 按钮启动/停止整段 session。
- 把相机和已连接 IMU 数据写入同一个 `data/raw/session_*` 目录。

## 边界

- Dashboard 只是采集层工具，不做 AprilTag / OpenVINS / ESKF 处理。
- Idle preview 直接读 camera；Record 中预览已写入磁盘的最新 JPEG，避免同一个 `/dev/video*` 被重复打开。
- 离线处理使用 `scripts/process_apriltag_session.py`、`scripts/prepare_p3_head_vio.py` 等脚本。
