# Scripts

Utility scripts will live here.

Planned scripts:

- dataset conversion
- JSONL validation
- trajectory export
- calibration sanity checks

Current scripts:

- `capture_dashboard.py`: local browser UI for camera preview, camera capture, IMU scan, and IMU capture.
- `capture_imu_jsonl.py`: WT-series BLE IMU capture without GUI.
- `list_cameras.py`: list V4L2 capture devices and supported formats.
- `check_camera_capture.py`: open selected cameras and write one probe image per camera before dashboard recording.
- `capture_quad_camera.py`: four-camera frame capture with host timestamps.
- `validate_session.py`: inspect one dashboard session and explain whether camera/IMU data was actually saved.
- `postprocess_session.py`: validate a recorded session and optionally run AprilTag/OpenVINS preparation in one command.
- `process_apriltag_session.py`: offline AprilTag wristband pose processing from recorded camera frames.
- `prepare_openvins_session.py`: validate one recorded dashboard session and export OpenVINS-friendly image/IMU JSONL streams.
- `write_openvins_rosbag2.py`: write prepared OpenVINS JSONL streams to rosbag2 when a ROS2 Python environment is sourced.
- `generate_openvins_config.py`: generate first-pass OpenVINS Kalibr-style config files from current camera calibration.
- `check_head_vio_readiness.py`: check whether one session is ready for the P3a `C0 + head_imu` OpenVINS path.
- `prepare_p3_head_vio.py`: prepare P3a OpenVINS JSONL streams and config in one offline step.

## P3 Head VIO

P3a uses one camera and one rigid-mounted head IMU:

```text
C0 + head_imu -> T_W_H
```

The prototype frame convention is:

```text
I = head_imu frame
H := I
T_W_H := T_W_I
```

Before preparing OpenVINS inputs, check the recorded session:

```bash
python scripts/check_head_vio_readiness.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

Then generate the OpenVINS intermediate streams and config:

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

This writes:

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/images.jsonl
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/imu.jsonl
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/config/
```

Convert to rosbag2 only from a sourced ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
python scripts/write_openvins_rosbag2.py \
  --prepared-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0 \
  --bag-dir data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/rosbag2 \
  --frame-id head_imu
```
