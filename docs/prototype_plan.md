# Prototype Plan

## P0: Wrist Visual Pose Only

Goal:

```text
fixed headset/camera frame -> stable T_H_B
```

Steps:

1. Mount one camera or the headset rigidly.
2. Run the current AprilTag ring tracker.
3. Move the bracelet through normal hand poses.
4. Log `T_H_B`, source tag IDs, reprojection error, and tracking state.

Pass criteria:

- static wrist pose jitter below 1-2 cm
- no large pose flip when visible tag changes
- recover after short occlusion

## P1: Wrist IMU Logging

Goal:

```text
wrist IMU + visual T_H_B recorded on a common timeline
```

Steps:

1. Add raw IMU logger for acceleration and gyroscope.
2. Store timestamps in microseconds.
3. Record visual wrist pose at camera rate.
4. Plot timestamp alignment and dropped frames.

Pass criteria:

- monotonic timestamps
- known camera/IMU offset or measured offset
- repeatable short-motion logs

## P2: Offline Wrist ESKF

Goal:

```text
IMU propagation corrected by AprilTag visual pose
```

Steps:

1. Implement a minimal offline ESKF.
2. Use visual pose as correction.
3. Simulate visual dropouts.
4. Measure drift and recovery.

Pass criteria:

- visual-tracked mode is stable
- 0.5-1.0 s dropout does not explode
- re-acquisition does not cause unacceptable jumps

## P3: Head VIO With OpenVINS

Goal:

```text
head cameras + head IMU -> T_W_H
```

Steps:

1. Start with one camera + one head IMU.
2. Add more headset cameras only after the basic pipeline is stable.
3. Export `T_W_H` into the same timestamp convention.

Pass criteria:

- stable boot origin
- no obvious scale failure
- acceptable short-range drift for action capture

## P4: Full Rigid-Body Output

Goal:

```text
T_W_B = T_W_H * T_H_B
```

Steps:

1. Combine head and wrist streams by timestamp.
2. Interpolate poses where needed.
3. Emit the WAM schema from `Context.md`.

Pass criteria:

- head and wrist poses share one world frame
- output schema is stable
- logs can be replayed offline

## P5: MOLA Integration

Goal:

```text
replayable, inspectable, extensible motion-capture pipeline
```

Steps:

1. Record rosbag2 datasets.
2. Add MOLA replay configs.
3. Use trajectory tools for evaluation.
4. Add environment AprilTag anchor grids if world drift needs correction.

