# Tests

Planned checks:

- transform composition correctness
- quaternion normalization
- timestamp monotonicity
- schema validation
- bracelet geometry consistency

Current automated checks:

- `test_session_pipeline.py`: no-hardware tests for dashboard session validation,
  partial camera failure handling, OpenVINS JSONL preparation, and postprocess
  skip behavior.

Run:

```bash
python -m pytest tests
```

Hardware checks still need to be run manually on the capture machine:

- dashboard Start/Stop with real cameras
- BLE scan/connect for two WT IMUs
- AprilTag detection on real wristband images
- ROS2 rosbag2 export/playback for OpenVINS
