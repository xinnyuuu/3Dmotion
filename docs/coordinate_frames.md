# Coordinate Frames and Transform Notation

This project uses rigid-body transforms between coordinate frames.

## Frames

- `W`: world frame. Origin is the headset boot pose in the first prototype.
- `H`: headset base frame. Center of the multi-camera headset rig.
- `C_i`: camera frame for the `i`th headset camera.
- `T_i`: local frame of the `i`th visible AprilTag on the bracelet.
- `B`: wristband rigid-body frame, located at the geometric center of the bracelet.

## What `T_W<-H` Means

`T_W<-H` means:

```text
a transform from H coordinates into W coordinates
```

If a point `p_H` is expressed in the headset frame, then:

```text
p_W = T_W<-H * p_H
```

Equivalent names:

```text
T_W<-H
T_W_H
pose of H in W
H-to-W transform
```

It is the 6DoF pose of the headset in the world.

## What `T_H<-B` Means

`T_H<-B` means:

```text
a transform from wristband coordinates B into headset coordinates H
```

If a point `p_B` is expressed in the wristband frame, then:

```text
p_H = T_H<-B * p_B
```

Equivalent names:

```text
T_H<-B
T_H_B
pose of B in H
B-to-H transform
```

It is the 6DoF pose of the wristband as seen by the headset camera rig.

## Composition

The final wristband pose in the world frame is:

```text
T_W<-B = T_W<-H * T_H<-B
```

In words:

1. OpenVINS estimates where the headset is in the world: `T_W<-H`.
2. AprilTag ring tracking estimates where the wristband is relative to the headset: `T_H<-B`.
3. Multiplying the two gives the wristband pose in the world: `T_W<-B`.

## Wristband From Visible Tag

When camera `C_i` sees tag `T_j`, the wrist center can be recovered with:

```text
T_H<-B = T_H<-C_i * T_C_i<-T_j * T_T_j<-B
```

For a multi-camera solver, the same unknown `T_H<-B` is optimized from all
visible cameras and all visible tag corners together.

