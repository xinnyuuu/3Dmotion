# head_vio_bridge

Purpose:

```text
OpenVINS output -> project head pose stream
```

Primary output:

```text
T_W_H
```

The current implementation covers P3a readiness checks, offline session
conversion, first-pass OpenVINS config generation, and rosbag2 export.

## P3a Frame Convention

The first head VIO prototype deliberately uses one camera and one IMU:

```text
C0 + head_imu -> T_W_H
```

`head_imu` must be rigidly fixed to the headset/camera rig. OpenVINS assumes
the camera and IMU belong to one rigid body.

For P3a:

```text
I = head_imu frame
H := I
T_W_H := T_W_I
```

After IMU-to-head calibration, replace the temporary convention with:

```text
T_W_H = T_W_I * T_I_H
```

## Current Prototype Entry

Check that one recorded session is ready for P3a:

```bash
python scripts/check_head_vio_readiness.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

Prepare the OpenVINS JSONL streams and first-pass config:

```bash
python scripts/prepare_p3_head_vio.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

This writes:

```text
data/processed/session_YYYYMMDD_HHMMSS/openvins_c0/
  images.jsonl
  imu.jsonl
  openvins_session_manifest.json
  p3_head_vio_summary.json
  config/
```

The lower-level ROS-free exporter is still available:

```bash
python scripts/prepare_openvins_session.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --camera-id C0 \
  --imu-slot head_imu \
  --output-dir data/processed/openvins_session
```

It checks that the recorded camera frames and head IMU stream exist, then maps
them to the first OpenVINS topic plan:

```text
C0 -> /cam0/image_raw
head_imu -> /imu0
```

The next bridge step is a rosbag2 writer that publishes these records as
`sensor_msgs/msg/Image` and `sensor_msgs/msg/Imu`.

That writer now exists, but it must be run from a ROS2 Python environment:

```bash
source /opt/ros/humble/setup.bash
python scripts/write_openvins_rosbag2.py \
  --prepared-dir data/processed/openvins_session \
  --bag-dir data/processed/openvins_session/rosbag2
```

This produces:

```text
/cam0/image_raw  sensor_msgs/msg/Image
/imu0            sensor_msgs/msg/Imu
```

OpenVINS should be launched separately against a matching estimator config.
Keep the first test monocular with `max_cameras:=1` and `use_stereo:=false`.

Generate the first-pass config from current camera calibration:

```bash
python scripts/generate_openvins_config.py \
  --cameras configs/cameras.yaml \
  --camera-id C0 \
  --output-dir configs/openvins/generated_head_vio
```

The generated `kalibr_imucam_chain.yaml` uses:

```text
T_imu_cam = T_H_C
```

That assumes the headset frame `H` is currently standing in for the head IMU
frame. It is enough to wire the pipeline, but not enough for final accuracy.
