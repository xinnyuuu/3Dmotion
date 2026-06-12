# apriltag_ring_node

Purpose:

```text
headset camera images -> bracelet visual pose observation
```

Primary output:

```text
T_H_B
```

This package should reuse the existing `AprilTag/ring.py` logic during the
first prototype, then become a ROS 2 node when the data path stabilizes.

Near-term plan:

1. Read frame groups from `quad_camera_capture`.
2. Run AprilTag detection per camera.
3. Estimate per-camera tag poses using camera intrinsics.
4. Transform observations into the headset frame with `T_H_Ci`.
5. Solve one shared `T_H_B` from all visible cameras and tag corners.

The existing `AprilTag` project already has useful pieces:

- tag detection
- tag pose estimation
- regular-hex bracelet center estimation
- target tag pose prediction from neighboring visible tags

The first integrated version can call those utilities directly before replacing
the single-camera pose selection with a true multi-camera optimizer.

