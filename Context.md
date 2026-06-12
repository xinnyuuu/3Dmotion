Context Profile: VIMAS-WAM (Action Capture Skill)

I would like to develop this project based on the mola.
1. Project Background & Objective
* Domain: Multi-modal Sensor Fusion, Rigid-Body 3D Motion Capture, World Action Model (WAM) Data Ingestion.
* Core Goal: Reconstruct the 6DoF high-fidelity rigid-body trajectories of the user's Head and Wrist into a unified 3D spatial coordinate system without anatomy-based skeleton/IK reconstruction. This serves as clean, raw physical token streams for downstream World Action Models.
* Hardware Setup:
    * Headset: 4x Global Shutter Fish-eye/Wide-angle cameras ($C_1$ to $C_4$: Left Ear, Front Left, Front Right, Right Ear) + Dual-IMU Array (Front/Back differential setup).
    * Wristband: A 6-sided hexagonal rigid bracelet embedded with non-repetitive AprilTag (Tag36h11) arrays on each surface + 1x Internal 9-axis IMU (ICM-42688-P).

---

2. Spatial-Geometric Geometry & Conventions

2.1 Coordinate Systems
1.  **Dynamic World Frame ($W$)**: Space origin $(0,0,0)$ anchored at the headset's initial boot-up pose.
2.  **Headset Base Frame ($H$)**: Spatial center of the 4-camera cluster.
3.  **Active Tag Frame ($T_i$)**: Local frame of the $i$-th visible AprilTag on the bracelet.
4.  **Wrist Rigid-Body Frame ($B$)**: Geometric center of the hexagonal wristband.

2.2 Mechanical Calibration Constraints
* Wristband Geometry: Flat-to-flat distance is denoted as $D$. The outer contour inradius is denoted as $r$. They satisfy the rigid constraint:
$$r = \frac{D}{2}$$
* Rigid Transform Dictionary: The homogeneous transformation matrix from any surface tag to the wrist center ($T_{T_i \leftarrow B}$) is a hard-coded geometric constant:
$$T_{T_i \leftarrow B} = \begin{bmatrix} R_i & \mathbf{t}_i \\ \mathbf{0}^T & 1 \end{bmatrix}, \quad \text{where } \|\mathbf{t}_i\| = r$$

2.3 Cascaded Transformation Equation
When any camera detects tag $$i$$, the 3D absolute position $$P_W$$ of the wrist center is resolved via:
$$P_W = T_{W \leftarrow H} \cdot T_{H \leftarrow T_i} \cdot T_{T_i \leftarrow B} \cdot P_B$$

---

3. Algorithmic Architecture & Implementation Pipeline

3.1 Multi-Camera Joint PnP Solver
* Mechanism: Instead of independent single-camera PnP threads, the system utilizes a centralized optimization backend. All 2D corners from visible cameras are warped into the headset frame $H$ using offline-calibrated extrinsics $T_{C_{idx} \leftarrow H}$.
* Cost Function (Ceres-Solver):
$$\arg\min_{T_{H \leftarrow B}} \sum_{i \in \text{Visible}} \sum_{j \in \text{Corners}} \left\| p_{2D, j}^{i} - \pi_i \left( T_{C_i \leftarrow H} \cdot T_{H \leftarrow B} \cdot P_{B, j} \right) \right\|_{\delta}^2$$
* Impact: Eradicates depth degeneration of single-camera tracking and ensures smooth cross-camera handovers.

3.2 Dual-EKF Cascaded Estimation Network
* Filter 1 (Head VIO): Resolves $T_{W \leftarrow H}$ via a standard Visual-Inertial Odometry pipeline using 4-cam environmental features and the differential Head Dual-IMU array (which cancels neck muscle artifacts).
* Filter 2 (Wrist Fusion): A 15-dimensional Error-State Kalman Filter tracking state:
$$\mathbf{x} = \begin{bmatrix} \mathbf{p}_W^T & \mathbf{v}_W^T & \mathbf{q}_W^T & \mathbf{b}_a^T & \mathbf{b}_g^T \end{bmatrix}^T$$
    * **Propagation ($200\text{ Hz}$)**: Hand strap IMU high-frequency dead reckoning.
    * **Correction ($60\text{ Hz}$)**: Triggered when Multi-Cam Joint PnP yields a valid $T_{H \leftarrow B}$ observation.

3.3 Gating & Degradation Logic
* VISUAL_TRACKED: Standard EKF update; updates IMU biases ($\mathbf{b}_a, \mathbf{b}_g$) online.
* LOST_DEAD_RECKONING: Triggered during blind spots (e.g., hand behind back). EKF falls back to pure inertial propagation with soft kinematic gating boundaries ($\|\mathbf{p}_{wrist} - \mathbf{p}_{head}\| \le \text{Arm\_Length}$).

---

4. Ground Truth & Validation Protocols
1.  Primary GT (Lab Environment): OptiTrack/Vicon Infrared Motion Capture system ($\le 0.5\text{ mm}$ error). Uses rigid marker bodies on the headset and wristband, time-synced via hardware TTL pulses. Evaluated using Absolute Trajectory Error (ATE) RMSE ($\le 1.5\text{ cm}$ target).
2.  Secondary GT (In-the-wild): Large AprilTag anchor grids mapped across the environment to ground-truth and constrain VIO drift.

---

5. Standard Ingestion Data Schema (for WAM Tokenizer)
The system outputs a high-frequency binary/JSON stream with the following immutable topology:
{
  "timestamp_us": 1718128030045123,
  "tracking_state": 1, // 0: LOST, 1: VISUAL_OK, 2: PURE_IMU
  "head_6dof": {
    "pos_w": [X, Y, Z],        // meters
    "rot_w": [qw, qx, qy, qz]  // Quaternion
  },
  "wrist_6dof": {
    "pos_w": [X, Y, Z],
    "rot_w": [qw, qx, qy, qz],
    "linear_vel_w": [Vx, Vy, Vz], // m/s
    "angular_vel_b": [Wx, Wy, Wz], // rad/s
    "linear_acc_b": [Ax, Ay, Az]   // Bias-corrected m/s^2
  }
}




