# Calibration Checklist

## Camera Intrinsics

Required per camera:

- resolution
- camera matrix
- distortion coefficients
- camera model

Stored in:

```text
configs/cameras.yaml
```

## Camera-to-Head Extrinsics

Required per camera:

```text
T_H_Ci or T_Ci_H
```

Choose one convention and document it in `configs/frames.yaml`.

## Head IMU-to-Head Extrinsics

Required:

```text
T_H_IMU_FRONT
T_H_IMU_BACK
```

For the first OpenVINS prototype, use one IMU first. Add dual-IMU logic after
single-IMU VIO is stable.

## Wrist IMU-to-Wristband Extrinsics

Required:

```text
T_B_IMU_WRIST
```

This is needed before wrist IMU propagation can be fused cleanly with visual
bracelet pose.

## Bracelet Geometry

Required:

- tag family
- tag IDs
- tag side length
- flat-to-flat distance
- tag order around bracelet
- transform from each tag frame to wristband frame

Stored in:

```text
configs/bracelet.yaml
```

## Time Offset Calibration

Required offsets:

- camera to head IMU
- wrist IMU to camera/head clock
- host receive latency if BLE timestamps are unavailable

