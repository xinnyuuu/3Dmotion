# 系统使用流程

这份文档是当前 3DMotion 四目相机流程的日常入口。主线目标是：

```text
四目头环相机 C0-C3
  + 桌面 AprilGrid tag36h11 ids 100-111
  + 手环六面 AprilTag tag16h5
  -> AprilGrid world anchor: T_W_H
  -> wrist world pose: T_W_B
  -> RViz 同时检查 head / wrist 轨迹
```

也就是说，当前 MVP 主线不再依赖单个 reference tag 或 OpenVINS 初始化。优先用桌面 AprilGrid 直接定义 world frame，输出 `world_anchor/motion_frame.jsonl`。ROS-free OpenVINS 仍保留为 head VIO 调试路径；ROS2 rosbag2 只作为需要 topic/RViz/OpenVINS ROS 节点联调时的备用路径。

## 0. 当前主线

推荐日常顺序：

```text
配置 Python 环境
-> 检查 configs/cameras.yaml 和 configs/bracelet.yaml
-> 启动 dashboard
-> 录制四目 raw session
-> validate session
-> process_world_anchor_session.py 生成 T_W_H / T_W_B / motion_frame.jsonl
-> motion_replay_rviz 同时检查 head / wrist 轨迹
```

仓库同时保留两种方法：

```text
方法 1 / experimental:
  OpenVINS sparse visual feature tracks + head_imu -> T_W_H
  不依赖 AprilGrid；当前因为标定、omni camera model、时间同步还在调试中，先保留为后续扩展。

方法 2 / current MVP:
  AprilGrid world frame + wrist AprilTag ring + optional IMU -> T_W_H / T_W_B
  这是当前交给同事复现和继续迭代的默认路线。
```

核心输入输出：

```text
data/raw/session_YYYYMMDD_HHMMSS/cameras/
  frames.jsonl
  C0/*.jpg
  C1/*.jpg
  C2/*.jpg
  C3/*.jpg

configs/cameras.yaml
configs/world_tags.yaml
configs/bracelet.yaml

data/processed/session_YYYYMMDD_HHMMSS/world_anchor/
  world_anchor_candidates.jsonl
  wrist_world_candidates.jsonl
  head_pose.jsonl
  motion_frame.jsonl
```

坐标系约定见 `docs/coordinate_frames.md`。这里最常用的是：

```text
T_W_H: headset/camera rig frame H in AprilGrid world frame W
T_W_B: wristband frame B in AprilGrid world frame W
```

## 0.1 用仓库示例数据复现

仓库包含 `session_20260630_141240` 的一个小摘录：

```text
data/examples/session_20260630_141240_excerpt/
```

它保留了四目图片、`frames.jsonl`、`head_imu.jsonl` 和 `wrist_imu.jsonl`，用于验证 AprilGrid world-anchor 主线。运行：

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate

python scripts/process_dashboard_session.py \
  data/examples/session_20260630_141240_excerpt \
  --hands
```

默认输出：

```text
data/processed/session_20260630_141240_excerpt/world_anchor/
  world_anchor_candidates.jsonl
  wrist_world_candidates.jsonl
  hand_skeletons.jsonl
  head_pose.jsonl
  motion_frame.jsonl
  motion_frame_filtered.jsonl
```

`motion_frame.jsonl` 是原始估计；`motion_frame_filtered.jsonl` 是数据层滤波后的结果，供 RViz 和后续算法评估使用，不是显示层 smoothing。

RViz replay：

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vimas_motion_bringup --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1

ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py \
  motion_jsonl:=$(realpath ../data/processed/session_20260630_141240_excerpt/world_anchor/motion_frame_filtered.jsonl)
```

## 1. 进入项目和环境

```bash
cd ~/lxy/3DMotion
source .venv/bin/activate
```

第一次配置 venv：

```bash
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

确认关键依赖：

```bash
python - <<'PY'
import cv2
print("opencv", cv2.__version__, "omnidir", hasattr(cv2, "omnidir"))
PY
```

当前 `configs/cameras.yaml` 使用 Mei/omni 相机模型和 `xi`，所以 AprilTag 离线处理需要 `cv2.omnidir` 可用。

## 2. 检查标定配置

先跑：

```bash
python scripts/check_calibration_readiness.py
```

四目 AprilTag 主线至少需要：

```text
configs/cameras.yaml
  C0-C3 intrinsics
  C0-C3 distortion
  C0-C3 xi
  C0-C3 T_H_C

configs/bracelet.yaml
  tag_family
  tag_size_m
  center_offset_m 或 flat_to_flat_m
  tag_order
  ring_offset_sign
  ring_order_direction
```

当前手环 tag 是：

```yaml
tag_order:
  - 19
  - 16
  - 17
  - 18
  - 15
  - 14
```

`tag_order` 必须是手环物理绕一圈的相邻顺序。起点可以换，但绕圈方向不能错。

如果手环中心被画到 tag 外侧，改：

```yaml
ring_offset_sign: 1
```

如果中心位置对，但手环姿态绕圈方向反了，改：

```yaml
ring_order_direction: -1
```

## 3. 启动采集 dashboard

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
- 选择 resolution / fps / format
- 预览四路 camera 画面
- 扫描并连接 `head_imu` / `wrist_imu`
- 同步开始和停止 session 录制
- 把相机帧和已连接 IMU 写入同一个 session 文件夹

四目 AprilTag 主线只强依赖 `cameras/`。IMU 可以同时录，但不是 `process_apriltag_session.py` 的必要输入。

## 4. 采集前检查

列出相机：

```bash
python scripts/list_cameras.py --configs
```

单相机 preflight：

```bash
python scripts/check_camera_capture.py \
  --source C0:/dev/video0 \
  --format MJPG \
  --width 1600 \
  --height 1200 \
  --fps 25 \
  --output-dir data/raw/camera_preflight
```

成功后应看到：

```text
data/raw/camera_preflight/C0_probe.jpg
data/raw/camera_preflight/camera_preflight_summary.json
```

IMU 有两种连接方式：

- **系统 BLE 直连**：电脑蓝牙直接连接 WT901BLE。适合调试和没有 USB adapter 时使用，但依赖 BlueZ/系统蓝牙状态。
- **WIT/BWT901 USB serial-adapter**：USB adapter 负责 BLE 连接，Python 只读写 `/dev/ttyACM*`。这是 dashboard 录制时的推荐方式。

系统 BLE 扫描：

```bash
python scripts/capture_imu_jsonl.py --scan
```

如果没有结果，用更宽的 BLE 扫描：

```bash
python scripts/capture_imu_jsonl.py --scan-all --scan-timeout-s 20
```

BLE 直连采集示例：

```bash
python scripts/capture_imu_jsonl.py --transport ble --address C4:65:91:2C:E2:20 --sensor-id head_imu --output data/raw/head_imu.jsonl
python scripts/capture_imu_jsonl.py --transport ble --address C5:64:B9:44:66:D6 --sensor-id wrist_imu --output data/raw/wrist_imu.jsonl
```

如果两个 IMU 分别接在 WIT/BWT901 USB serial-adapter 上，先确认每只适配器能看到对应模块：

```bash
python scripts/capture_imu_jsonl.py --adapter-scan --serial-port /dev/ttyACM0
python scripts/capture_imu_jsonl.py --adapter-scan --serial-port /dev/ttyACM1
```

如果 `/dev/ttyACM*` 报 `Permission denied`：

```bash
sudo usermod -a -G dialout "$USER"
```

执行后需要退出当前桌面/终端重新登录。临时测试可以用：

```bash
sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1
```

然后一只适配器固定连一只模块。当前实测默认映射是 `head_imu -> /dev/ttyACM1`，`wrist_imu -> /dev/ttyACM0`：

```bash
python scripts/capture_imu_jsonl.py --transport serial-adapter --serial-port /dev/ttyACM1 --address C4:65:91:2C:E2:20 --sensor-id head_imu --output data/raw/head_imu.jsonl
python scripts/capture_imu_jsonl.py --transport serial-adapter --serial-port /dev/ttyACM0 --address C5:64:B9:44:66:D6 --sensor-id wrist_imu --output data/raw/wrist_imu.jsonl
```

串口适配器默认使用 `--timestamp-mode reconstructed-rate --sample-rate-hz 200`。也就是说，`timestamp_monotonic_ns` / `timestamp_unix_ns` 是按 200 Hz nominal rate 从串口批量接收时间反推出来的采样时间；原始主机接收时间保存在 `host_receive_monotonic_ns` / `host_receive_unix_ns`。如果需要和旧数据做 A/B，可临时加 `--timestamp-mode host-receive`。

Dashboard 使用同一套后台采集代码。前端只负责选择设备、发起连接和显示状态；IMU 数据由后端 worker 持续采集。打开或刷新页面不会自动重连 IMU，启动后需要在 IMU 区域点击 `连接所选` 或 `连接两个 IMU`。录制主流程可这样启动，后续仍然写入 `session/imus/head_imu.jsonl` 和 `session/imus/wrist_imu.jsonl`：

```bash
python scripts/capture_dashboard.py
```

如果连接后出现 `waiting_data` / `no_data`，先确认没有其它进程占用串口，再断开重连；WIT adapter 进入半连接状态时需要物理拔插 USB adapter。

当前常用 WT901BLE 地址：

```text
C4:65:91:2C:E2:20
C5:64:B9:44:66:D6
```

## 5. 录制四目 session

在 dashboard 里确认：

- `C0-C3` 都有清晰预览
- AprilTag 在至少一路相机里清晰可见
- 录制格式和 `configs/cameras.yaml` 的 `image_size` 对应
- 输出目录使用默认 `data/raw/session_YYYYMMDD_HHMMSS`

建议动作：

```text
0-2s: 手环静止，让四目看到稳定 tag
2-10s: 手腕缓慢平移和旋转，保证多个 tag 轮流可见
10-20s: 做目标动作，但避免 tag 长时间完全遮挡
最后 1-2s: 静止
```

一次 session 结构应该类似：

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

如果只做四目 AprilTag，`imus/` 可以缺失或为空。

## 6. Session 检查

基础检查：

```bash
python scripts/validate_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

项目级质量检查：

```bash
python project_tests/session_quality/check_session_quality.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

重点看：

- `overall_ok`
- `frames.jsonl` 是否存在且非空
- `C0-C3` 是否都有保存图片
- camera timestamp 是否单调；当前相机仍是 `host_retrieve` 软件时间戳
- IMU `timestamp_source` 是否为 `reconstructed_200hz_from_host_receive`，并保留 `host_receive_*` 原始接收时间
- 四目 frame group 是否稳定
- 四目 `skew_us` 是否过大
- `capture_errors.jsonl` 是否大量报错

## 7. 离线处理当前 AprilGrid world-anchor 主线

最推荐的一条命令是：

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands
```

它会读取 dashboard 保存的相机配置，验证 session，运行 AprilGrid world-anchor pipeline，并打印 RViz replay 命令。

如果想显式控制输出目录，用底层入口：

```bash
python scripts/process_world_anchor_session.py offline \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --world-tags configs/world_tags.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/world_anchor \
  --hands
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/world_anchor/
  world_anchor_candidates.jsonl   # per-camera/per-tag board/head candidates
  wrist_world_candidates.jsonl    # per-camera/per-tag wrist candidates
  hand_skeletons.jsonl            # optional approximate hand landmarks
  head_pose.jsonl                 # T_W_H
  motion_frame.jsonl              # T_W_H + T_W_B + relative state
  world_anchor_summary.json
```

手部 landmark 默认优先使用多目三角化；只有最近有稳定多目结果时，单目检测才作为 guided fallback。严格只看多目结果：

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands \
  --hand-multiview-only
```

## 8. 旧版 wrist-only AprilTag 路径

这条路径只输出相对头环的 `T_H_B`，不会建立 AprilGrid world frame。它保留用于 A/B debug 和未来 OpenVINS route 的 wrist visual measurement，不是当前推荐入口。

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

含义：

```text
wrist_visual_candidates.jsonl
  每个 camera/tag 独立得到的一条 T_H_B candidate

wrist_visual_pose.jsonl
  同一个 group_id 下多个 candidate 按 reprojection error 加权融合后的 T_H_B
```

当前融合方式是离线原型：每个 tag/camera 先独立 PnP，再按重投影误差加权平均。后续可以升级成真正的 multi-camera joint PnP。

## 9. wrist-only 输出 sanity check

看输出行数：

```bash
wc -l \
  data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_candidates.jsonl \
  data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_pose.jsonl
```

统计识别到的 tag、camera 和重投影误差：

```bash
python - <<'PY'
import json
from collections import Counter
from pathlib import Path

p = Path("data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_candidates.jsonl")
tags = Counter()
cameras = Counter()
errs = []

with p.open() as f:
    for line in f:
        if not line.strip():
            continue
        r = json.loads(line)
        tags[r["tag_id"]] += 1
        cameras[r["camera_id"]] += 1
        errs.append(r["reprojection_error_px"])

print("tags", dict(sorted(tags.items())))
print("cameras", dict(sorted(cameras.items())))
if errs:
    print("reproj mean/min/max", sum(errs) / len(errs), min(errs), max(errs))
PY
```

正常情况下：

- `tags` 里只应该出现 `configs/bracelet.yaml` 中的六个 id
- `cameras` 应该覆盖实际参与录制的相机
- 平均重投影误差通常应在几个像素以内；当前好数据可低于 `1 px`
- `wrist_visual_pose.jsonl` 不为空

看第一条 fused pose：

```bash
head -n 1 data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_pose.jsonl
```

## 10. 手环方向验证

这一步最重要。新的 tag id 或贴法变了以后，不要只看“能检测到”，还要看方向是否连续。

推荐验证动作：

```text
1. 手环静止，让 1-2 个 tag 稳定可见。
2. 缓慢绕手环周向转动，让相邻 tag 依次进入视野。
3. 保持手环中心大致不动，观察 tag 切换时 T_H_B 是否突跳。
```

判断：

- tag 切换时位置不应突然跳到手环另一侧。
- 姿态不应突然翻转约 180 度。
- 如果中心点总在 tag 外侧，改 `ring_offset_sign`。
- 如果中心点对，但绕圈姿态方向反，改 `ring_order_direction`。
- 如果只有某一个 tag 切到另一个 tag 时跳，优先检查 `tag_order` 物理顺序。

`tag_order` 的规则：

```text
必须按手环相邻面绕一圈写。
起点可以任意。
方向要和 ring_order_direction 配合。
```

例如下面两个只是不同起点，等价：

```yaml
[19, 16, 17, 18, 15, 14]
[15, 14, 19, 16, 17, 18]
```

但如果物理方向相反，就需要改 `ring_order_direction` 或反转列表。

## 11. 常见 AprilGrid / AprilTag 问题

`cv2.omnidir` 不存在：

```text
当前 configs/cameras.yaml 是 Mei/omni 模型，AprilTag 处理需要 OpenCV contrib 的 omnidir 模块。
确认使用的是项目 .venv。
```

检查：

```bash
source .venv/bin/activate
python - <<'PY'
import cv2
print(cv2.__version__, hasattr(cv2, "omnidir"))
PY
```

输出为空：

- 原图里 tag 是否清晰、完整、无遮挡。
- `configs/bracelet.yaml` 的 `tag_family` 是否是 `tag16h5`。
- `tag_size_m` 是否和打印 tag 实际边长一致。
- `tag_order` 是否包含当前手环所有 id。
- `configs/cameras.yaml` 的 `image_size` 是否和录制分辨率一致。
- `frames.jsonl` 里的图片路径是否存在。

重投影误差很大：

- 相机内参和录制分辨率不匹配。
- tag 尺寸填错。
- 图像运动模糊。
- tag 太小或太斜。
- camera `T_H_C` 外参不准会影响融合后的 `T_H_B`，但单个 candidate 的 reprojection error 主要看 PnP 和内参。

某些 tag 数量明显偏少：

- 对应 tag 被手或夹具挡住。
- 对应 tag 贴歪、反光、打印损坏。
- 手环动作让它很少朝向相机。

## 12. OpenVINS experimental：ROS-free head VIO

这条路线是方法 1：不依赖 AprilGrid，使用 sparse visual feature tracks + `head_imu` 估计 `T_W_H`。代码保留给后续扩展；当前 MVP 复现不需要先跑 OpenVINS。

项目脚本会把 dashboard session 整理成 OpenVINS config、camera CSV 和 IMU CSV，然后直接让 `run_csv_msckf` 读取原始 JPG 和 CSV，生成 `head_pose.jsonl`。这条路径不依赖 ROS2，也不会生成巨大的 raw image rosbag2。

先确认 runner 存在且可执行：

```bash
ls -l /home/lxy/lxy/VIO_data/VIO/open_vins/ov_msckf/build_local/run_csv_msckf
chmod +x /home/lxy/lxy/VIO_data/VIO/open_vins/ov_msckf/build_local/run_csv_msckf
```

你的这次报错：

```text
PermissionError: [Errno 13] Permission denied: .../run_csv_msckf
```

对应的就是 `run_csv_msckf` 文件有读权限但没有执行权限。`ls -l` 如果显示 `-rw-r--r--`，需要上面的 `chmod +x`；修好后应类似 `-rwxr-xr-x`。

日常一条命令：

```bash
cd ~/lxy/3DMotion

python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_head/
  images.jsonl
  imu.jsonl
  config/
    estimator_config.yaml
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
  rosfree/
    inputs/
      cam0.csv
      cam1.csv
      cam2.csv
      cam3.csv
      imu.csv
    openvins_trajectory.csv
    rosfree_inputs_summary.json
    rosfree_run_summary.json
  head_pose.jsonl
  head_vio_rosfree_summary.json
```

默认相机：

```text
C1 -> cam0
C2 -> cam1
C0 -> cam2
C3 -> cam3
```

`VIO_data` 里的当前 `run_csv_msckf.cpp` 支持 `cam0` 到 `cam3`，要求 camera 参数从 `cam0` 开始连续。默认四目可以直接跑；如果要先缩小问题，推荐先跑前向双目：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C1 \
  --camera-id C2
```

如果只想单目快速排查：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C1
```

快速调试可以截取小窗口：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 12 \
  --image-stride 2
```

这样只会少读一些 JPG，不会写 rosbag2。如果只想准备 CSV/config，不运行 OpenVINS：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --prepare-only
```

检查 head pose：

```bash
wc -l data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
head -n 1 data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
cat data/processed/session_YYYYMMDD_HHMMSS/openvins_head/rosfree/rosfree_run_summary.json
```

如果流程在运行 OpenVINS 前就失败，优先看错误类型：

- `Permission denied: .../run_csv_msckf`：runner 没有执行权限，运行 `chmod +x /home/lxy/lxy/VIO_data/VIO/open_vins/ov_msckf/build_local/run_csv_msckf`。
- `ROS-free OpenVINS runner does not exist`：`--runner` 路径不对，或 `VIO_data` 里的 OpenVINS 没有 build。
- `No ROS-free image records for cameras`：选择的 `--camera-id` 在这个 session 里没有图像，或 camera ID 和 `configs/cameras.yaml` 不一致。

如果 `head_pose.jsonl` 为空，先看 `rosfree_run_summary.json` 里的 `stdout_tail` / `stderr_tail`。常见情况：

- `OpenVINS exited normally but wrote no states`：runner 完成了，但 OpenVINS 没初始化。
- `failed static init` / `platform moving too much`：录制开头没有足够静止，或图像 disparity 太大。
- `no accel jerk detected`：静止后激励不明显，或截取窗口不包含有效运动。

优先尝试：

```bash
python scripts/process_head_vio_session_rosfree.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --start-offset-s 2 \
  --max-duration-s 20
```

如果仍然没有 trajectory，检查：

- `head_imu` 是否刚性固定在头环/四目结构上。
- 开头是否有 2-3 秒静止。
- 静止后是否有足够平移和加速度变化。
- C1/C2/C0/C3 图像是否清晰、有纹理。
- IMU 单位是否是 `m/s^2` 和 `rad/s`。
- camera/IMU 时间偏移和 `T_imu_cam` 是否太粗。

## 13. OpenVINS experimental：ROS2 rosbag2 / RViz head VIO 调试

这条路径用于验证 OpenVINS ROS2 node、ROS topic、TF 或 RViz 实时显示。当前 ROS2 head VIO 调试包是 **四目 + head_imu**：

```text
C1,C2,C0,C3 -> /cam0/image_raw ... /cam3/image_raw
head_imu    -> /imu0
```

`wrist_imu` 不参与 OpenVINS head VIO；它在第 13 节 motion fusion 阶段和 `wrist_visual_pose.jsonl` 一起用于手腕姿态传播/校正。也就是说，这里不是四目 + 2 IMU 的 rosbag2，而是四目 + 1 个刚性固定在头环上的 head IMU。它会把 JPG 解码成 raw `sensor_msgs/Image` 写进 rosbag2，文件会明显变大，所以更适合调试，不作为日常主线。

Step 1：准备 OpenVINS config / JSONL，并写 rosbag2。

```bash
cd ~/lxy/3DMotion

scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-duration-s 8 \
  --image-stride 2
```

默认也用四目和 `head_imu`：`C1,C2,C0,C3` -> `/cam0/image_raw` 到 `/cam3/image_raw`，`head_imu` -> `/imu0`。如果只想生成双目调试 bag：

```bash
scripts/process_head_vio_session_ros2.bash \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C1 \
  --camera-id C2 \
  --max-duration-s 8 \
  --image-stride 2
```

输出核心文件：

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_head/
  config/estimator_config.yaml
  images.jsonl
  imu.jsonl                 # head_imu only, exported as /imu0
  rosbag2/
  rosbag2_summary.json
```

Step 2：用三个 terminal 跑 OpenVINS、播放 bag、导出 RViz pose。

Terminal A：启动 OpenVINS ROS2 node。

```bash
cd ~/lxy/3DMotion
source scripts/source_openvins_ros2.bash

ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(pwd)/data/processed/session_YYYYMMDD_HHMMSS/openvins_head/config/estimator_config.yaml \
  max_cameras:=4 \
  use_stereo:=false
```

如果上一步只生成了双目 bag，把 `max_cameras:=4` 改成 `max_cameras:=2`。

Terminal B：播放刚生成的 rosbag2。

```bash
cd ~/lxy/3DMotion
source /opt/ros/humble/setup.bash

ros2 bag play data/processed/session_YYYYMMDD_HHMMSS/openvins_head/rosbag2
```

Terminal C：启动 RViz，并把 OpenVINS 头部轨迹导出成 `head_pose.jsonl`。

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vimas_motion_bringup head_vio_rviz.launch.py \
  child_frame:=head_imu \
  output_jsonl:=/home/lxy/lxy/3DMotion/data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
```

RViz 应该显示：

```text
/motion/head_pose
/motion/head_path
TF: world -> head_imu
```

注意这里看的是 head VIO 输出。最终同时看 head 和 wrist 的 RViz replay 走第 13-14 节：先用 `head_pose.jsonl + wrist_visual_pose.jsonl + imus/wrist_imu.jsonl` 融合成 `motion_frame.jsonl`，再启动 `motion_replay_rviz.launch.py`。

检查导出的 head pose：

```bash
wc -l data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
head -n 1 data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl
```

如果 `head_pose.jsonl` 为空，说明 OpenVINS 没有发布 `/ov_msckf/poseimu`，或 RViz bridge 启动时 topic/type 不匹配。先查：

```bash
ros2 topic list
ros2 topic info /ov_msckf/poseimu
ros2 topic hz /imu0
ros2 topic hz /cam0/image_raw
```

## 14. OpenVINS experimental：融合 head 和 wrist

有了：

```text
openvins_head/head_pose.jsonl       T_W_H
wrist_visual/wrist_visual_pose.jsonl T_H_B
imus/wrist_imu.jsonl                optional wrist IMU
```

就可以生成：

```text
motion/wrist_fused_pose.jsonl
motion/motion_frame.jsonl
```

运行：

```bash
cd ~/lxy/3DMotion

python -m packages.session_tools.motion_fusion \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --output-root data/processed/session_YYYYMMDD_HHMMSS \
  --head-pose data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl \
  --wrist-visual data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_pose.jsonl \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/motion
```

默认会启用 wrist IMU gyro 姿态融合：

```text
wrist gyro propagation + AprilTag visual correction
```

含义：

- `wrist_imu` 的 `gyro_radps` 用来在相邻 AprilTag visual pose 之间预测 wrist orientation。
- AprilTag visual pose 每帧作为绝对校正，防止 gyro 漂移。
- wrist translation 仍来自 AprilTag visual pose。
- `motion_frame.jsonl` 里的 wrist angular velocity / acceleration 也来自最近的 wrist IMU 样本。

如果要退回纯视觉，使用：

```bash
python -m packages.session_tools.motion_fusion \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --output-root data/processed/session_YYYYMMDD_HHMMSS \
  --head-pose data/processed/session_YYYYMMDD_HHMMSS/openvins_head/head_pose.jsonl \
  --wrist-visual data/processed/session_YYYYMMDD_HHMMSS/wrist_visual/wrist_visual_pose.jsonl \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/motion \
  --disable-wrist-imu-orientation
```

输出：

```text
data/processed/session_YYYYMMDD_HHMMSS/motion/
  wrist_fused_pose.jsonl
  motion_frame.jsonl
  fusion_summary.json
```

检查：

```bash
cat data/processed/session_YYYYMMDD_HHMMSS/motion/fusion_summary.json
wc -l data/processed/session_YYYYMMDD_HHMMSS/motion/motion_frame.jsonl
```

重点看：

```text
counts.motion_frames > 0
alignment.max_head_dt_ms 不要过大
skipped.head_time_gap 不要接近 wrist_visual_pose 总数
wrist_imu_fusion.imu_propagated_frames > 0 表示 wrist gyro 确实参与了姿态预测
```

`motion_frame.jsonl` 里每一帧同时包含：

```text
head:  T_W_H
wrist: T_W_B
relative.T_H_B
fusion.method
```

## 15. RViz 同时检查头部和手部轨迹

当前仓库已有 `motion_replay_visualizer.py`，它直接读取 `motion_frame.jsonl`，发布 RViz topic 和 TF：

```text
/motion/head_pose
/motion/head_path
/motion/wrist_pose
/motion/wrist_path
TF: world -> head
TF: head -> wrist
```

运行：

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vimas_motion_bringup motion_replay_rviz.launch.py \
  motion_jsonl:=/home/lxy/lxy/3DMotion/data/processed/session_YYYYMMDD_HHMMSS/motion/motion_frame.jsonl \
  loop:=true \
  rate_hz:=30.0
```

RViz 判断标准：

- Head Path 应连续，不应大幅抖动或瞬移。
- Wrist Path 应跟随 head 坐标变化，不应在 tag 切换时翻到另一侧。
- `world -> head -> wrist` 的 TF 方向应合理。
- 如果 head 正常、wrist 跳，优先查 `tag_order` / `ring_order_direction` / `ring_offset_sign`。
- 如果 head 自己漂或初始化失败，回到 ROS-free OpenVINS 的 `rosfree_run_summary.json` 和 `head_pose.jsonl` 阶段查 head VIO；需要 topic 级排查时再走 ROS2 rosbag2 分支。

注意：目前最终 head/wrist 联合检查是 JSONL replay 到 RViz，不是把 `motion_frame.jsonl` 再写成 rosbag2。日常主线也不再把 OpenVINS 输入打包成 rosbag2；ROS-free runner 直接读取 JPG/CSV。只有 ROS2 调试分支会生成这些 raw image topic：

```text
/cam0/image_raw
/cam1/image_raw
/cam2/image_raw
/cam3/image_raw
/imu0
```

如果后续需要把最终 `/motion/head_pose`、`/motion/wrist_pose`、TF 和 path 也固化成 rosbag2，需要新增一个 `write_motion_rosbag2.py`。

## 16. 一条命令 smoke test

如果只是想快速跑当前 AprilGrid world-anchor MVP，可以用：

```bash
python scripts/process_dashboard_session.py \
  data/raw/session_YYYYMMDD_HHMMSS \
  --hands
```

等价分步命令：

```bash
python scripts/process_world_anchor_session.py offline \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --world-tags configs/world_tags.yaml \
  --bracelet configs/bracelet.yaml \
  --output-dir data/processed/session_YYYYMMDD_HHMMSS/world_anchor \
  --hands
```

ROS2 只在最终 `motion_replay_rviz` 可视化阶段需要。

## 17. 最终组合关系

当前模块关系：

```text
AprilGrid world anchor:
  C0-C3 + desktop tag36h11 ids 100-111 -> T_W_H

AprilTag wrist visual:
  C0-C3 + wrist tag16h5 ring -> T_W_B

OpenVINS head VIO:
  ROS-free JPG/CSV + head_imu -> T_W_H
  可选 ROS2 调试: /cam0..3/image_raw + /imu0(head_imu) -> T_W_H
```

MOLA 目前不在 dashboard 采集和四目 AprilTag 主线里。它后续用于 replay、trajectory tools、map/anchor constraints。

## 18. 目录速查

```text
configs/        标定、手环、OpenVINS 配置
docs/           设计和操作文档
packages/       Python package 代码
scripts/        常用 CLI 入口
data/raw/       dashboard 采集的原始 session，git 忽略
data/processed/ 离线处理结果，git 忽略
project_tests/ 真实 session 的质量检查工具
open_vins/      外部 OpenVINS 仓库
```
