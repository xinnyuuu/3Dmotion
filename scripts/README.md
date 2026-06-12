# Scripts

Utility scripts will live here.

Planned scripts:

- dataset conversion
- JSONL validation
- trajectory export
- calibration sanity checks

Current scripts:

- `capture_imu_jsonl.py`: WT-series BLE IMU capture without GUI.
- `list_cameras.py`: list V4L2 capture devices and supported formats.
- `capture_quad_camera.py`: four-camera frame capture with host timestamps.
- `process_apriltag_session.py`: offline AprilTag wristband pose processing from recorded camera frames.
