# 系统使用流程

这份文档把当前 3DMotion 原型的配置、采集、检查和离线处理流程放在一起。日常使用时先看这里；需要更细的设计细节再跳到其它文档。

相关文档入口见 `docs/README.md`。

当前推荐路线是：

```text
配置环境
-> 启动 dashboard
-> 采集 raw session
-> 检查 session 质量
-> 离线处理 AprilTag / OpenVINS
-> 再做融合和导出
```

先不要把所有模块塞进实时闭环。dashboard 只负责稳定采集，后处理脚本负责重复离线调试。

## 0. 项目目录

进入项目：

```bash
cd ~/lxy/3DMotion
```

核心目录：

```text
configs/        标定、传感器、OpenVINS 配置
docs/           设计和操作文档
packages/       Python package 代码
scripts/        常用 CLI 入口
data/raw/       dashboard 采集的原始 session，git 忽略
data/processed/ 离线处理结果，git 忽略
project_tests/ 真实 session 的质量检查工具
open_vins/      外部 OpenVINS 仓库
```

当前硬件角色：

```text
C0-C3       四个头环 camera
head_imu    固定在头环刚体上的 IMU
wrist_imu   固定在手环上的 IMU
```

## 1. Python 环境

Python venv 用于 dashboard、相机采集、BLE IMU 采集、AprilTag 离线处理和数据转换。

第一次配置：

```bash
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

以后每个新终端如果要跑 Python 脚本，先执行：

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate
```

确认 CLI 可用：

```bash
python scripts/capture_dashboard.py --help
python scripts/capture_imu_jsonl.py --help
python scripts/capture_quad_camera.py --help
```

## 2. ROS2 / OpenVINS 环境

ROS2 和 OpenVINS 不强行放进 Python venv。它们在 ROS2 终端里跑。

安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rosbag2-transport \
  libboost-all-dev \
  libceres-dev \
  libeigen3-dev
```

构建 OpenVINS：

```bash
cd ~/lxy/3DMotion/open_vins
source /opt/ros/humble/setup.bash

colcon build \
  --event-handlers console_cohesion+ \
  --packages-select ov_core ov_init ov_msckf ov_eval \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

以后启动 OpenVINS 前，在 3DMotion 根目录使用 helper：

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash
```

如果终端里出现：

```text
ros2: command not found
```

说明当前终端没有 source ROS2：

```bash
source /opt/ros/humble/setup.bash
```

构建本项目 ROS2 workspace：

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 3. 启动采集前端

启动 dashboard：

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate
python scripts/capture_dashboard.py
```

浏览器打开：

```text
http://127.0.0.1:8766/
```

dashboard 当前负责：

- 选择 `C0-C3` 对应的 `/dev/video*`
- 选择相机支持的 resolution / fps / format
- 预览 camera 画面
- 扫描 BLE IMU
- 为 `head_imu` 和 `wrist_imu` 选择对应 BLE 设备
- 用一个 Record 按钮启动/停止整段 session 采集
- 把相机和已连接 IMU 数据写到同一个 session 文件夹

录制中的预览来自已经写入磁盘的最新 JPEG，所以看到的画面更接近实际保存的数据。

## 4. 采集前检查

如果某个相机打不开，先用 preflight 单独测：

```bash
python scripts/list_cameras.py --configs
python scripts/check_camera_capture.py \
  --source C0:/dev/video0 \
  --format MJPG \
  --width 1600 \
  --height 1200 \
  --fps 25 \
  --output-dir data/raw/camera_preflight
```

成功后会写：

```text
data/raw/camera_preflight/C0_probe.jpg
data/raw/camera_preflight/camera_preflight_summary.json
```

BLE IMU 可单独扫描：

```bash
python scripts/capture_imu_jsonl.py --scan
```

如果两个 IMU 名字相同，要按 BLE address 区分。dashboard 里选择到 `head_imu` 或 `wrist_imu` 后，就用对应 address 连接。

如果 `--scan` 没结果，先看 Linux / BlueZ 是否能扫到任何 BLE 广播：

```bash
python scripts/capture_imu_jsonl.py --scan-all --scan-timeout-s 12
```

`--scan` 只显示看起来像 WT IMU 的设备；`--scan-all` 会列出所有 BLE 设备，适合排查设备是否在广播、是否被过滤掉。

目前用的两个IMU地址
```bash
WT901BLE67	C4:65:91:2C:E2:20	rssi=None	services=
WT901BLE67	C5:64:B9:44:66:D6	rssi=None	services=
```

## 5. 采集一段 session

在 dashboard 里确认：

- 至少 `C0` 有清晰预览
- `head_imu` 已连接
- 如果要测试手环，则 `wrist_imu` 已连接
- 输出目录使用默认 `data/raw/session_YYYYMMDD_HHMMSS`

点击 Record 开始，再点击同一个按钮停止。

一次 session 的原始结构应该类似：

```text
data/raw/session_YYYYMMDD_HHMMSS/
  session_manifest.json
  session_summary.json
  cameras/
    frames.jsonl
    capture_errors.jsonl
    C0/00000000.jpg
    C1/00000000.jpg
    C2/00000000.jpg
    C3/00000000.jpg
  imus/
    head_imu.jsonl
    wrist_imu.jsonl
```

未连接的设备可以缺失或为空。原型期允许只记录可用设备，但后处理脚本会按目标任务检查必要输入。

## 6. P3a 推荐采集动作

如果目标是测试 OpenVINS head VIO，采集一段 20-30 秒数据：

```text
0-3s: 静止，让 IMU 初始化重力和 bias
3-10s: 慢速左右/前后平移 0.3-1.0m
10-18s: 小幅转头或转动头环
18-25s: 轻微平移 + 转动
最后 2s: 静止
```

注意：

- `head_imu` 必须刚性固定在头环/四目结构上。
- 第一版 OpenVINS 只用 `C0 + head_imu`。
- 不要对着白墙，画面里要有纹理。
- 不要全程静止，否则 OpenVINS 可能报 `failed static init: no accel jerk detected`。

## 7. Session 质量检查

采集后先跑基础检查：

```bash
python scripts/validate_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

dashboard 停止录制后也会自动写：

```text
data/raw/session_YYYYMMDD_HHMMSS/session_summary.json
```

再跑项目级质量检查：

```bash
python project_tests/session_quality/check_session_quality.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

重点看：

- `overall_ok`
- IMU timestamp 是否单调
- camera frame group 是否稳定
- 四目 `skew_us` 是否过大
- camera 和 IMU 是否都有足够时长

如果要专门测试 P3a：

```bash
python scripts/check_head_vio_readiness.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C0 \
  --imu-slot head_imu
```

这个检查会额外判断 `head_imu` 加速度变化是否足够让 OpenVINS 初始化。

## 8. AprilTag 手环离线处理

目标：

```text
head cameras observe wrist AprilTags -> T_H_B
```

运行：

```bash
python scripts/process_apriltag_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS/cameras \
  --cameras configs/cameras.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/wrist_visual
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/
  wrist_visual_candidates.jsonl
  wrist_visual_pose.jsonl
```

如果输出为空，优先检查：

- 画面里是否真的看到了 AprilTag
- `configs/bracelet.yaml` 里的 tag IDs 和 tag size 是否正确
- `configs/cameras.yaml` 的 camera intrinsics / distortion 是否正确
- 图片路径和 `frames.jsonl` 是否匹配

## 9. OpenVINS P3a 一条命令处理

目标：

```text
C0 + head_imu -> T_W_H
```

当前 P3a 临时 frame 约定：

```text
I_H = head_imu frame
H := I_H
T_W_H := T_W_IH
```

推荐日常使用这一条命令。它会自动做：

```text
P3a readiness check
-> 导出 OpenVINS images.jsonl / imu.jsonl
-> 生成 OpenVINS config
-> 写 rosbag2
-> 写 head_vio_process_summary.json
```

最稳的方式是用包装脚本。它会先用 `.venv` Python 做项目数据处理，再自动 source ROS2，用 `/usr/bin/python3` 写 rosbag2，避免 venv 和 ROS2 Python 包互相看不到：

```bash
cd ~/lxy/3DMotion

scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

如果只是想快速看一次 RViz 轨迹，建议先生成小一点的 debug bag：

```bash
scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 8 \
  --image-stride 2
```

含义：

- `--max-duration-s 8`: 只导出 8 秒窗口。
- `--image-stride 2`: camera 每 2 张取 1 张，IMU 样本仍然全保留。
- 这个不会改变图像尺寸，所以不会破坏当前 camera intrinsics。

如果你的当前 Python 环境同时能看到项目依赖和 ROS2 Python 包，也可以直接用 Python 版：

```bash
source /opt/ros/humble/setup.bash

python scripts/process_head_vio_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 8 \
  --image-stride 2
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/
  images.jsonl
  imu.jsonl
  openvins_session_manifest.json
  p3_head_vio_summary.json
  head_vio_process_summary.json
  config/
    estimator_config.yaml
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
  rosbag2/
    rosbag2_0.db3
    metadata.yaml
```

如果当前终端只是普通 Python venv，没有 ROS2，也可以先只生成 JSONL 和 config：

```bash
source .venv/bin/activate

python scripts/process_head_vio_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --no-rosbag2
```

之后在 ROS2 终端补写 rosbag2：

```bash
source /opt/ros/humble/setup.bash

python scripts/process_head_vio_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

重复处理同一个 session 时，脚本默认会覆盖旧的 `rosbag2/`。如果想保留旧 bag，用：

```bash
python scripts/process_head_vio_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --keep-existing-rosbag2
```

处理完成后看总报告：

```bash
cat data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/head_vio_process_summary.json
```

注意：写入 rosbag2 时会把 JPEG 解码成 ROS `sensor_msgs/Image` 的 `bgr8` 原始图像，体积会比原始 JPG 大很多。20-30 秒、1080p 的测试 session 可能生成数 GB 的 bag。快速调试时优先用 `--max-duration-s` 和 `--image-stride`，或者采集时选择 720p / 15fps。

## 10. OpenVINS P3a 拆分调试路径

如果一条命令失败，或者你想看是哪一步有问题，可以拆开跑。

只准备 OpenVINS 中间数据和配置：

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C0 \
  --imu-slot head_imu
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/
  images.jsonl
  imu.jsonl
  openvins_session_manifest.json
  p3_head_vio_summary.json
  config/
    estimator_config.yaml
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
```

这一步不需要 ROS2。它只是把 dashboard session 转成 OpenVINS 友好的中间格式。

单独写 rosbag2：

```bash
cd ~/lxy/3DMotion
source /opt/ros/humble/setup.bash

python scripts/write_openvins_rosbag2.py \
  --prepared-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0 \
  --bag-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2 \
  --frame-id head_imu
```

检查 bag：

```bash
ros2 bag info data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2
```

应看到：

```text
/cam0/image_raw
/imu0
```

## 11. 跑 OpenVINS

用两个终端。

Terminal A：启动 OpenVINS。

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash

ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(pwd)/data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/config/estimator_config.yaml \
  max_cameras:=1 \
  use_stereo:=false
```

Terminal B：播放 rosbag2。

```bash
cd ~/lxy/3DMotion
source /opt/ros/humble/setup.bash

ros2 bag play data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2
```

如果 OpenVINS 一直初始化失败，优先检查：

- `head_imu` 是否真的固定在头环刚体上
- session 是否有开头静止 2-3 秒
- 后面是否有足够平移和加速度变化
- C0 图像是否清晰、有纹理
- IMU 单位是否是 `m/s^2` 和 `rad/s`
- camera 与 IMU 时间偏移是否过大
- `T_imu_cam` 是否还是粗略 identity

## 11.5 标定配置 readiness 检查

在四目 wrist visual、head VIO 或 wrist fusion 之前，先检查本仓库 `configs/`
里的标定值是否齐全：

```bash
cd ~/lxy/3DMotion
python scripts/check_calibration_readiness.py
```

当前检查项包括：

```text
configs/cameras.yaml          camera intrinsics/distortion/T_H_C
configs/frames.yaml           T_B_IB and frame conventions
configs/imu_calibration.yaml  head_imu/wrist_imu noise and bias
configs/bracelet.yaml         wristband tag geometry
```

VimasCalibration 是独立标定仓库。它产出相机/IMU 标定结果；3DMotion 只读取本仓库
`configs/` 中已经人工审阅并填好的 YAML，不直接从 VimasCalibration 自动写入。

如果 OpenVINS 进程刚启动就 `exit code -11`，通常不是数据运动问题，而是 OpenVINS config schema / YAML 与当前 OpenVINS 版本不匹配。先重新生成 P3a config：

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C0 \
  --imu-slot head_imu \
  --template-config-dir open_vins/config/euroc_mav
```

再重新写 rosbag2 并启动 OpenVINS。

## 12. RViz 可视化 Head VIO

OpenVINS 成功初始化后会发布 `/ov_msckf/poseimu`。本项目的 RViz bridge 会把它转成：

```text
/motion/head_pose
/motion/head_path
TF: world -> head_imu
```

启动第三个 terminal：

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vimas_motion_bringup head_vio_rviz.launch.py
```

如果 OpenVINS pose topic 不是 `/ov_msckf/poseimu`，先查：

```bash
ros2 topic list
ros2 topic info /ov_msckf/poseimu
```

然后指定 topic：

```bash
ros2 launch vimas_motion_bringup head_vio_rviz.launch.py \
  input_pose_topic:=/实际/topic \
  input_type:=pose_with_covariance_stamped
```

支持的 `input_type`：

```text
pose_with_covariance_stamped
pose_stamped
odometry
```

## 13. 全量 smoke test 后处理

如果想同时试 AprilTag 和 OpenVINS 的旧 smoke test，可以用：

```bash
python scripts/postprocess_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --apriltag \
  --openvins \
  --openvins-config
```

它会生成：

```text
data/processed/session_YYYYMMDD_HHMMSS/postprocess_summary.json
```

这条命令适合检查多个模块是否能跑通，但它不会写 rosbag2。现在如果只做 head VIO，优先使用 `scripts/process_head_vio_session.py`。

## 14. EuRoC Stereo 基准验证

如果要确认 3DMotion 当前 ROS2/OpenVINS/RViz 链路本身没问题，可以用 `VIO_data` 里的 EuRoC sample 作为 benchmark。

先生成 EuRoC stereo rosbag2：

```bash
cd ~/lxy/3DMotion

scripts/process_euroc_openvins_stereo_ros2.bash \
  --mav0-dir ../VIO_data/VIO/V2_01_easy/mav0 \
  --max-duration-s 20
```

完整 `V2_01_easy` 会生成较大的 raw image bag。第一次验证建议先用 `--max-duration-s 20`，确认链路通了再跑完整数据。

输出：

```text
data/processed/euroc_v2_01_easy/openvins_stereo/
  images.jsonl
  imu.jsonl
  config/
    estimator_config.yaml
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
  rosbag2/
```

用三个 terminal 跑。

Terminal A：启动 OpenVINS stereo：

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash

ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(pwd)/data/processed/euroc_v2_01_easy/openvins_stereo/config/estimator_config.yaml \
  max_cameras:=2 \
  use_stereo:=true
```

Terminal B：播放 EuRoC bag：

```bash
cd ~/lxy/3DMotion
source /opt/ros/humble/setup.bash

ros2 bag play data/processed/euroc_v2_01_easy/openvins_stereo/rosbag2
```

Terminal C：启动 RViz 轨迹可视化：

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vimas_motion_bringup head_vio_rviz.launch.py \
  child_frame:=euroc_imu
```

如果这条 EuRoC benchmark 正常，但头环数据漂移大，优先检查头环数据的：

- `T_imu_cam` 外参
- camera-IMU time offset
- IMU noise 参数
- camera intrinsics 与实际录制 resolution 是否一致

## 15. 输出关系

当前各模块的关系：

```text
OpenVINS:
  C0 + head_imu -> T_W_H

AprilTag wrist visual:
  headset cameras + wrist tag ring -> T_H_B

最终组合:
  T_W_B = T_W_H * T_H_B
```

后续 wrist ESKF 会把：

```text
wrist_imu + wrist_visual_pose -> smoother T_H_B / T_W_B
```

MOLA 放在更后面，用于 replay、trajectory tools、map/anchor constraints，不是当前 dashboard 采集的前置条件。

## 16. 常见问题

`python: command not found`

使用：

```bash
source .venv/bin/activate
```

或者直接用：

```bash
python3 ...
```

Dashboard 扫 IMU 报 `Install bleak to scan BLE IMU devices`

说明启动 dashboard 的 Python 环境没有安装 `bleak`。先确认你是从项目 venv 启动：

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/capture_dashboard.py
```

也可以直接检查：

```bash
python -c "import bleak; print(bleak.__version__)"
```

`ros2: command not found`

使用：

```bash
source /opt/ros/humble/setup.bash
```

`ROS2 Python packages are not available`

写 rosbag2 时没有正确 source ROS2，或者当前 Python 环境看不到 ROS2 包。先尝试不用 venv：

```bash
source /opt/ros/humble/setup.bash
python scripts/write_openvins_rosbag2.py ...
```

`ioctl(VIDIOC_QBUF): Bad file descriptor`

通常是 camera device 已经被关闭、被另一个进程抢占，或者 preview/record 竞争同一个 `/dev/video*`。先关掉占用相机的程序，再用 `check_camera_capture.py` 单独测试。

AprilTag 输出为空

先看原图里 tag 是否清晰可见，再检查 `configs/bracelet.yaml` 的 tag ID 和边长，以及 `configs/cameras.yaml` 的内参。

OpenVINS 报 `no accel jerk detected`

这通常说明 OpenVINS 在初始化窗口里没有检测到足够明显的加速度变化。当前 P3a config 会把 `init_imu_thresh` 调低到 `0.5`，先重新生成配置再试：

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C0 \
  --imu-slot head_imu
```

如果仍然失败，重新采一段：开头静止 2-3 秒，然后做 0.3-1.0m 的平移和轻微转动，动作要比纯转头更有线加速度。
