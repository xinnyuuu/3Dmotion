# Scripts

这里放命令行入口。完整操作顺序见 `../docs/system_workflow.md`。

## 当前脚本

- `capture_dashboard.py`: 本地浏览器 UI，用于 camera preview、camera capture、IMU scan 和 IMU capture。
- `capture_imu_jsonl.py`: 无 GUI 的 WT BLE IMU 采集。
- `list_cameras.py`: 列出 V4L2 设备和支持的 format / resolution / fps。
- `check_camera_capture.py`: 采集前 camera preflight，写一张 probe image 和 summary。
- `capture_quad_camera.py`: 四目 frame group 采集。
- `validate_session.py`: 检查 dashboard session 是否真的保存了 camera / IMU 数据。
- `postprocess_session.py`: 验证 session，并可选运行 AprilTag / OpenVINS 准备。
- `process_apriltag_session.py`: 从 recorded camera frames 离线生成 wrist visual pose。
- `process_head_vio_session.py`: P3a 一条命令后处理，生成 OpenVINS JSONL、config，并在 ROS2 环境可用时写 rosbag2。
- `process_head_vio_session_ros2.bash`: 更稳的 P3a 一条命令包装脚本，先用 `.venv` 处理项目数据，再用 ROS2 Python 写 rosbag2。
- `prepare_euroc_openvins_session.py`: 把 EuRoC `mav0` 数据转成 3DMotion OpenVINS prepared layout。
- `process_euroc_openvins_stereo_ros2.bash`: EuRoC stereo 一条命令处理，生成 rosbag2，用于 3DMotion OpenVINS/RViz 基准验证。
- `prepare_openvins_session.py`: 低层 OpenVINS JSONL exporter。
- `prepare_p3_head_vio.py`: P3a 一站式准备脚本，生成 OpenVINS JSONL 和 config。
- `check_head_vio_readiness.py`: 判断 session 是否适合 `C0 + head_imu` OpenVINS 测试。
- `generate_openvins_config.py`: 从当前 camera calibration 生成第一版 OpenVINS config。
- `check_calibration_readiness.py`: 检查 `configs/` 中 camera / IMU / bracelet 标定是否足够进入验证。
- `write_openvins_rosbag2.py`: 在 ROS2 环境里把 OpenVINS JSONL 写成 rosbag2。
- `source_openvins_ros2.bash`: source ROS2 Humble 和本地 OpenVINS package setup。

## Head VIO debug bag

1080p 的 `sensor_msgs/Image` rosbag2 会很大。快速看 RViz 轨迹时建议：

```bash
scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 8 \
  --image-stride 2
```

## EuRoC stereo benchmark

用 3DMotion 的 ROS2/OpenVINS/RViz pipeline 跑 EuRoC stereo：

```bash
scripts/process_euroc_openvins_stereo_ros2.bash \
  --mav0-dir ../VIO_data/VIO/V2_01_easy/mav0 \
  --max-duration-s 20
```

生成：

```text
data/processed/euroc_v2_01_easy/openvins_stereo/
```

## 后续骨架

- trajectory export
- final WAM motion stream export
