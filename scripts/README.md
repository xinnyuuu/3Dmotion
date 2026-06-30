# Scripts

这里放命令行入口。完整操作顺序见 `../docs/system_workflow.md`。

## 当前脚本

- `capture_dashboard.py`: 本地浏览器 UI，用于 camera preview、camera capture、IMU scan 和 IMU capture。
- `capture_imu_jsonl.py`: 无 GUI 的 WT BLE IMU 采集。
- `check_imu_live.py`: 终端连接 IMU，短时采集 JSONL 并立刻检查采样率、timestamp gap、accel/gyro 质量。
- `list_cameras.py`: 列出 V4L2 设备和支持的 format / resolution / fps。
- `check_camera_capture.py`: 采集前 camera preflight，写一张 probe image 和 summary。
- `capture_quad_camera.py`: 四目 frame group 采集。
- `validate_session.py`: 检查 dashboard session 是否真的保存了 camera / IMU 数据。
- `process_dashboard_session.py`: 当前推荐入口。读取 dashboard session，运行 AprilGrid world-anchor，默认生成 `motion_frame.jsonl` 和 `motion_frame_filtered.jsonl`，并打印 RViz replay 命令。
- `postprocess_session.py`: 底层编排入口，可选运行 AprilGrid world-anchor / legacy AprilTag / ROS-free head VIO / motion fusion。
- `process_world_anchor_session.py`: 当前 MVP 视觉主线，用桌面 AprilGrid `tag36h11` ids `100-111` 和手环 AprilTag 输出 `world_anchor/motion_frame.jsonl`。
- `filter_motion_frame.py`: 对 world-anchor `motion_frame.jsonl` 做数据层滤波，输出 `motion_frame_filtered.jsonl`。
- `process_apriltag_session.py`: legacy wrist-only 入口，从 recorded camera frames 离线生成相对 wrist visual pose `T_H_B`。
- `process_head_vio_session_rosfree.py`: 日常 head VIO 主线，生成 OpenVINS JSONL/config、ROS-free CSV 输入、trajectory 和 `head_pose.jsonl`。
- `process_head_vio_session.py`: P3a 后处理底层入口，生成 OpenVINS JSONL、config，并可选写 rosbag2。
- `process_head_vio_session_ros2.bash`: ROS2 调试入口，先用 `.venv` 处理项目数据，再用 ROS2 Python 写 rosbag2。
- `prepare_euroc_openvins_session.py`: 把 EuRoC `mav0` 数据转成 3DMotion OpenVINS prepared layout。
- `process_euroc_openvins_stereo_ros2.bash`: EuRoC stereo 一条命令处理，生成 rosbag2，用于 3DMotion OpenVINS/RViz 基准验证。
- `prepare_openvins_session.py`: 低层 OpenVINS JSONL exporter。
- `prepare_p3_head_vio.py`: P3a 一站式准备脚本，生成 OpenVINS JSONL 和 config。
- `check_head_vio_readiness.py`: 判断 session 是否适合 selected head cameras + `head_imu` OpenVINS 测试。
- `generate_openvins_config.py`: 从当前 camera calibration 生成第一版 OpenVINS config。
- `check_calibration_readiness.py`: 检查 `configs/` 中 camera / IMU / bracelet 标定是否足够进入验证。
- `write_openvins_rosbag2.py`: 在 ROS2 环境里把 OpenVINS JSONL 写成 rosbag2，仅用于 ROS2 调试分支。
- `replay_openvins_session_ros2.py`: 在 ROS2 环境里直接发布 OpenVINS JSONL/JPG 到 `/cam*/image_raw` 和 `/imu0`，不写 rosbag2，并记录 `/ov_msckf/poseimu` 到 `head_pose.jsonl`。
- `session_replay_publisher.py`: `replay_openvins_session_ros2.py` 的同名兼容入口。
- `session_replay_publisher_ros2.bash`: source ROS2/OpenVINS 后用 `/usr/bin/python3` 运行 replay publisher，避免 Conda Python 缺少 ROS2 包。
- `head_vio_rviz_ros2.bash`: source ROS2/OpenVINS/`ros2_ws` 后启动 head VIO RViz 可视化。
- `source_openvins_ros2.bash`: source ROS2 Humble 和本地 OpenVINS package setup。

## Live IMU Quality Check

录制前先用 terminal 采一小段 IMU 并检查时间戳质量。当前常用 head IMU 串口适配器：

```bash
python scripts/check_imu_live.py \
  --transport serial-adapter \
  --serial-port /dev/ttyACM1 \
  --address C4:65:91:2C:E2:20 \
  --sensor-id head_imu \
  --duration-s 10
```

wrist IMU：

```bash
python scripts/check_imu_live.py \
  --transport serial-adapter \
  --serial-port /dev/ttyACM0 \
  --address C5:64:B9:44:66:D6 \
  --sensor-id wrist_imu \
  --duration-s 10
```

如果不确定地址或 adapter index：

```bash
python scripts/capture_imu_jsonl.py --adapter-scan --serial-port /dev/ttyACM1
python scripts/check_imu_live.py --transport serial-adapter --serial-port /dev/ttyACM1 --adapter-device-index 0
```

VIO 用的 head IMU 希望看到 `PASS`，并且 `rate_hz` 接近 200、`dt_ms p99` 小于 10-20ms、`gaps_fail` 为 0。输出 JSONL 默认保存在 `/tmp/3dmotion_imu_check_*.jsonl`。

默认不轮询磁力计/四元数寄存器，只检查 VIO 需要的 accel/gyro 连续流；如果确实要检查 aux 数据，再加 `--aux-poll`。

## Head VIO ROS-free

这是 AprilGrid-free 的 future/experimental route，不是当前 MVP 复现的必需步骤。

日常 head VIO 不生成 rosbag2，直接读取 JPG/CSV：

```bash
chmod +x /home/lxy/lxy/VIO_data/VIO/open_vins/ov_msckf/build_local/run_csv_msckf

python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

默认相机顺序是 `C1,C2,C0,C3`，对应 OpenVINS `cam0..cam3`。如果先缩小到前向双目：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C1 \
  --camera-id C2
```

快速调试可截取小窗口：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 12 \
  --image-stride 2
```

## Head VIO ROS2 debug bag

1080p 的 `sensor_msgs/Image` rosbag2 会很大。只有需要 ROS2 topic/RViz/OpenVINS node 调试时再生成：

```bash
scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 8 \
  --image-stride 2
```

双目调试 bag：

```bash
scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C1 \
  --camera-id C2 \
  --max-duration-s 8 \
  --image-stride 2
```

## Head VIO ROS2 direct replay

更快的 ROS2/OpenVINS 调试路径是不写 rosbag2，只准备 JSONL/config，然后直接 replay raw session：

```bash
python scripts/process_head_vio_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --no-rosbag2

source scripts/source_openvins_ros2.bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=data/processed/session_YYYYMMDD_HHMMSS/openvins_head/config/estimator_config.yaml \
  max_cameras:=4 use_stereo:=false
```

另开一个 terminal 启动 RViz：

```bash
scripts/head_vio_rviz_ros2.bash
```

再开一个 terminal 发布 raw session：

```bash
scripts/session_replay_publisher_ros2.bash \
  --prepared-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_head
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
data/processed/session_YYYYMMDD_HHMMSS/openvins_head/session_replay_summary.json
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
