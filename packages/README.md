# Packages

这里放当前 Python 原型模块。完整使用流程见 `../docs/system_workflow.md`，系统结构见 `../docs/architecture.md`。

## 当前模块

- `capture_dashboard`: 本地浏览器采集前端。
- `quad_camera_capture`: 四目 frame group 采集和相机 preflight。
- `imu_ble_bridge`: WT BLE IMU 扫描、连接、解析和 JSONL 记录。
- `apriltag_ring_node`: 手环 AprilTag 离线检测，输出 `T_H_B` 视觉观测。
- `head_vio_bridge`: P3a OpenVINS session 准备、配置生成和 rosbag2 导出。
- `session_tools`: session 验证和一条命令后处理。
- `wrist_eskf`: 手环 IMU + 视觉位姿融合骨架。
- `wam_token_writer`: 最终 WAM motion stream 输出骨架。

## 维护原则

模块 README 只说明职责、输入输出和当前状态。具体操作命令集中维护在 `docs/system_workflow.md`，避免多处文档漂移。
