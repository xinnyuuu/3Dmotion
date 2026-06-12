# Data Schema

The final stream follows the topology described in `Context.md`.

## JSONL Debug Format

Each line is one timestamped state.

```json
{
  "timestamp_us": 1718128030045123,
  "tracking_state": 1,
  "head_6dof": {
    "pos_w": [0.0, 0.0, 0.0],
    "rot_w": [1.0, 0.0, 0.0, 0.0]
  },
  "wrist_6dof": {
    "pos_w": [0.0, 0.0, 0.0],
    "rot_w": [1.0, 0.0, 0.0, 0.0],
    "linear_vel_w": [0.0, 0.0, 0.0],
    "angular_vel_b": [0.0, 0.0, 0.0],
    "linear_acc_b": [0.0, 0.0, 0.0]
  }
}
```

## Tracking State

```text
0 = LOST
1 = VISUAL_OK
2 = PURE_IMU
```

## Intermediate Streams

Recommended intermediate logs:

```text
timestamp_us, sensor, ax, ay, az, gx, gy, gz, qw, qx, qy, qz
timestamp_us, frame, x, y, z, qw, qx, qy, qz, covariance_hint
timestamp_us, camera_id, tag_id, corners, reprojection_error
```

## Timestamp Rule

Use sensor timestamps whenever possible. If unavailable, use host receive time
and record that the timestamp source is `host_receive`.

Never use wall-clock time inside estimation logic once replay begins.

