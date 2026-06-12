# Timestamp Synchronization Strategy

The prototype should separate three timestamp concepts:

1. `timestamp_monotonic_ns`: host monotonic receive/retrieve time.
2. `timestamp_unix_ns`: host wall-clock time for file correlation.
3. `timestamp_device_ns`: sensor/device time, if the hardware provides it.

Use `timestamp_monotonic_ns` for fusion during early prototyping because it is
stable and does not jump if system time changes.

## Current Practical Recommendation

For the first 15-30 FPS prototype:

- Cameras: timestamp immediately after `retrieve()`.
- Four cameras: call `grab()` on all cameras first, then `retrieve()` each one.
- IMU BLE: timestamp immediately when the notification callback receives bytes.
- Store both `time.monotonic_ns()` and `time.time_ns()`.
- Record `timestamp_source`, e.g. `host_retrieve` or `host_receive`.

This gives approximate sync good enough to debug data flow and wrist pose
stability. It is not a final high-precision synchronization solution.

## Four-Camera Skew

The current `quad_camera_capture` groups four frames into one `group_id`.
For each group, it records:

```text
skew_us = camera_timestamp - group_center_timestamp
```

Use this to see whether the four camera streams are close enough. If the skew
is regularly above a few milliseconds, the visual solver may still work at slow
motions but fast hand motion will show artifacts.

## IMU-to-Camera Offset

BLE IMU timestamps will initially be host receive timestamps, so there is
latency and jitter. Estimate the offset empirically:

1. Attach the wrist IMU rigidly to an AprilTag board or bracelet.
2. Perform sharp rotations visible to the camera.
3. Compare gyro magnitude peaks against visual angular velocity peaks.
4. Fit a constant offset first.
5. Only add more complex clock models if the residual drift demands it.

## Final System Recommendation

For production-quality capture, aim for this order:

1. Hardware-triggered global-shutter cameras.
2. Shared trigger line or sync pulse into camera and IMU logger.
3. Device timestamps from camera and IMU.
4. Host receive timestamps only as diagnostics.

If hardware sync is unavailable, use software sync with explicit uncertainty:

- estimate constant camera-IMU offset
- interpolate IMU to camera timestamps
- inflate measurement covariance based on observed jitter
- reject frame groups with excessive camera skew

## Rule for Estimation Code

Do not call wall-clock time inside the estimator. Estimators should consume
timestamps from records/messages only, so live capture and offline replay behave
the same way.

