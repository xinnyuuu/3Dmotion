# Architecture

The prototype is split into small modules so each sensor path can be tested
independently.

## Data Flow

```text
head cameras + head IMU
  -> head_vio_bridge
  -> /motion/head_pose

head cameras + bracelet AprilTags
  -> apriltag_ring_node
  -> /motion/wrist_visual_pose

wrist IMU
  -> imu_ble_bridge
  -> /motion/wrist_imu

/motion/wrist_imu + /motion/wrist_visual_pose + /motion/head_pose
  -> wrist_eskf
  -> /motion/wrist_pose

/motion/head_pose + /motion/wrist_pose
  -> wam_token_writer
  -> JSONL/binary WAM motion stream
```

## Module Responsibilities

### `head_vio_bridge`

Wraps OpenVINS output into the project pose format.

Inputs:

- camera images
- head IMU
- camera/IMU calibration

Output:

- `T_W_H`

### `apriltag_ring_node`

Detects wristband AprilTags and estimates the wristband rigid-body pose.

Inputs:

- headset camera images
- camera intrinsics
- camera-to-head extrinsics
- bracelet geometry

Output:

- `T_H_B` visual observation

### `imu_ble_bridge`

Converts WT-series BLE IMU packets into timestamped project or ROS 2 messages.

Inputs:

- BLE packets

Outputs:

- acceleration
- angular velocity
- optional device quaternion
- device timestamp if available, host timestamp otherwise

### `wrist_eskf`

Fuses wrist IMU propagation with AprilTag visual corrections.

State:

```text
p_W, v_W, q_W, b_a, b_g
```

Outputs:

- smoothed wrist 6DoF pose
- linear velocity
- bias-corrected acceleration
- tracking state

### `wam_token_writer`

Serializes head and wrist motion into the stable downstream data schema.

Output:

- JSONL for debugging
- binary stream later if needed

## MOLA Role

MOLA is not the fastest first-choice VIO core for this project. Use it later for:

- rosbag2 replay
- sensor-pipeline configuration
- trajectory tools
- map or AprilTag anchor-grid constraints
- visualization and evaluation support

## OpenVINS Role

OpenVINS is the recommended first VIO core for fast head-pose iteration:

- mature visual-inertial odometry
- multi-camera support
- ROS integration
- camera/IMU calibration workflow

