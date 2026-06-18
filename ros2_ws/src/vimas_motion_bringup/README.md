# vimas_motion_bringup

Launch files and runtime composition for the 3D Motion prototype.

## Head VIO RViz

After OpenVINS is running and publishing `/ov_msckf/poseimu`, launch:

```bash
cd ~/lxy/3DMotion/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vimas_motion_bringup head_vio_rviz.launch.py
```

The visualizer republishes the current P3a head pose as:

```text
/motion/head_pose
/motion/head_path
TF: world -> head_imu
```

P3a treats `head_imu` as the temporary head frame until IMU-to-head extrinsics
are calibrated.
