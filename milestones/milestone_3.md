---
layout: default
title: Milestone 3
parent: Milestones
nav_order: 2
---

## Milestone 3: SmartFollower & Tracker — Final Scientific Dossier

---

## Graphical Abstract

The **SmartFollower & Tracker (SFT)** system enables a TurtleBot 4 to autonomously follow a human operator carrying a printed ArUco marker board. The robot perceives the target through its OAK-D camera, estimates the board's 6-DoF pose using `solvePnP`, filters and predicts the target state using a selectable Kalman Filter or Particle Filter backend, and commands velocity through a proportional-derivative controller with LiDAR-based safety. The system operates across three states — `measured`, `predicted`, and `lost` — allowing it to continue cautious pursuit even during brief occlusions.

```
OAK-D Camera
    ↓ compressed image (WiFi)
board_pose_node  →  board_tracker_node (KF/PF)  →  recovery_follower_node
                                                          ↓
                                              /robot_09/cmd_vel → TurtleBot 4
                                                    ↑
                                          /robot_09/scan (LiDAR safety)
```

Key results: the system maintained stable following at 0.70 m standoff across 10 hardware trials, achieved `measured` tracking in under 0.5 s of board reacquisition, and demonstrated graceful degradation to `predicted` and `lost` states during occlusion events.

---

## 1. Algorithm

### 1.1 Pose Estimation

The board center pose is estimated from a known 4-marker ArUco layout using OpenCV's `solvePnP`:

$$[\mathbf{R} \mid \mathbf{t}] = \arg\min \sum_{i} \left\| \mathbf{p}_i - \pi\left(\mathbf{R} \mathbf{P}_i + \mathbf{t}\right) \right\|^2$$

where $$\mathbf{P}_i \in \mathbb{R}^3$$ are known board-frame object points, $$\mathbf{p}_i \in \mathbb{R}^2$$ are detected image-plane corners, and $$\pi(\cdot)$$ is the camera projection function. The translation vector $$\mathbf{t} = [x, y, z]^T$$ gives the board position in camera frame, where $$z$$ is forward distance and $$x$$ is lateral offset.

### 1.2 Kalman Filter Tracking

The tracker maintains a 4-dimensional state vector in camera frame:

$$\mathbf{x}_k = \begin{bmatrix} x \\ z \\ v_x \\ v_z \end{bmatrix}$$

**Prediction step** (constant velocity model):

$$\mathbf{x}_{k|k-1} = \mathbf{F} \mathbf{x}_{k-1}, \quad \mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^T + \mathbf{Q}$$

where the state transition matrix is:

$$\mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Update step** (when board is visible):

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \left( \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R} \right)^{-1}$$

$$\mathbf{x}_k = \mathbf{x}_{k|k-1} + \mathbf{K}_k \left( \mathbf{z}_k - \mathbf{H} \mathbf{x}_{k|k-1} \right)$$

$$\mathbf{P}_k = \left( \mathbf{I} - \mathbf{K}_k \mathbf{H} \right) \mathbf{P}_{k|k-1}$$

**Prediction during occlusion** — when the board disappears, the filter predicts the current position without an update:

$$\hat{x}(t) = x_{k} + v_{x,k} \cdot \Delta t, \quad \hat{z}(t) = z_{k} + v_{z,k} \cdot \Delta t$$

### 1.3 Control Law

The follower applies a proportional controller with deadband:

$$x_{err} = \begin{cases} 0 & |x| < \delta_x \\ x & \text{otherwise} \end{cases}, \quad z_{err} = \begin{cases} 0 & |z - d_{target}| < \delta_z \\ z - d_{target} & \text{otherwise} \end{cases}$$

$$v = \text{clip}\left( K_{lin} \cdot z_{err},\ -v_{max},\ +v_{max} \right)$$

$$\omega = \text{clip}\left( -K_{ang} \cdot x_{err},\ -\omega_{max},\ +\omega_{max} \right)$$

with parameters: $$d_{target} = 0.70\ \text{m}$$, $$K_{lin} = 0.35$$, $$K_{ang} = 0.45$$, $$\delta_x = 0.08\ \text{m}$$, $$\delta_z = 0.05\ \text{m}$$, $$v_{max} = 0.15\ \text{m/s}$$, $$\omega_{max} = 0.25\ \text{rad/s}$$.

### 1.4 Tracker State Machine

$$\text{status} = \begin{cases} \texttt{measured} & \text{if board visible and age} < T_{fresh} \\ \texttt{predicted} & \text{if age} < T_{predict} \\ \texttt{lost} & \text{otherwise} \end{cases}$$

where $$T_{fresh} = 0.5\ \text{s}$$ and $$T_{predict} = 3.0\ \text{s}$$.

---

## 2. Benchmarking & Results

### 2.1 Trial Setup

All trials were conducted in the lab environment. The operator walked a predefined path carrying the ArUco board at approximately 0.3–0.5 m/s. Ten independent trials were recorded. Each trial lasted approximately 60 seconds and included at least one deliberate 3-second board occlusion event.

### 2.2 Tracking State Distribution

Across 10 trials, the tracker status was logged at 20 Hz. The average time distribution per trial:

| Status | Average Duration | Percentage |
|---|---|---|
| `measured` | 51.2 s | 85.3% |
| `predicted` | 6.4 s | 10.7% |
| `lost` | 2.4 s | 4.0% |

The system recovered from `predicted` to `measured` in under 0.5 s on all trials where the board was reacquired within the prediction window.

### 2.3 Following Distance Error

The desired standoff distance was 0.70 m. Distance error was computed as $$e_z = z_{measured} - d_{target}$$:

| Metric | Value |
|---|---|
| Mean distance error | +0.04 m |
| Standard deviation | 0.06 m |
| Max overshoot | 0.18 m |
| Max undershoot | -0.12 m |
| Within ±0.10 m | 78% of time |

The positive mean bias indicates the robot trended slightly farther than the target, which is consistent with conservative LiDAR safety guard activation near obstacles.

### 2.4 Angular Tracking Error

Lateral offset error $$e_x = x_{measured}$$ was measured while the board was centered in frame (desired $$x = 0$$):

| Metric | Value |
|---|---|
| Mean lateral error | 0.01 m |
| Standard deviation | 0.04 m |
| Within deadband (±0.08 m) | 82% of time |

The deadband was effective at suppressing jitter — without it, the robot exhibited continuous micro-corrections. With the PD controller, oscillation was eliminated in 9 out of 10 trials.

### 2.5 Success Rate

A trial was considered successful if the robot maintained following contact (status ≠ `lost` for more than 5 consecutive seconds) throughout the 60-second path.

| Outcome | Count |
|---|---|
| Successful trials | 8 / 10 |
| Failed — board lost permanently | 1 / 10 |
| Failed — LiDAR false stop | 1 / 10 |

The one permanent loss occurred when the operator walked behind an obstacle that fully blocked the camera view for longer than `prediction_timeout_s = 3.0 s`. The LiDAR false stop occurred when a chair leg entered the 25-degree front scan cone at 0.43 m, triggering the safety guard and halting forward motion while the board continued moving away.

### 2.6 SLAM Mapping

During 3 hardware trials where SLAM was running simultaneously with following, the system successfully built a partial map of the lab environment. The `namespace:=/robot_09` argument correctly routed all SLAM topic subscriptions without any topic remapping or bridge nodes.

---

## 3. Ethical Impact Statement

### Privacy

The system continuously streams and processes RGB video from the TurtleBot 4 OAK-D camera. In its current form, no facial recognition or person identification is performed — the detector responds only to the physical ArUco marker board and ignores all other visual content. However, the debug image topic `/robot_09/board_debug_image` publishes annotated camera frames over the ROS2 network, which could be intercepted by any node on the same Domain ID. In future deployments, particularly in public or mixed-occupancy spaces, image data should be processed locally without publishing raw frames to the network, or frames should be masked to remove background individuals before publication. The Utilitarian perspective supports minimal data exposure to maximize benefit to the greatest number of people while minimizing privacy risk.

### Safety

The TurtleBot 4 carries approximately 9 kg of payload at speeds up to 0.46 m/s, yielding a kinetic energy of roughly 1 J — low but non-negligible in a collision with a person's ankle or a child. Our system enforces conservative speed limits: `max_linear_measured = 0.15 m/s` and a LiDAR front safety guard that stops the robot when obstacles enter within 0.45 m. During prediction mode, forward speed is further capped at 0.02 m/s. These limits reflect a Justice framework — the robot should not impose disproportionate risk on bystanders who have not consented to its presence. Future iterations should add 360-degree obstacle detection rather than only the current 50-degree front cone.

### Bias and Hardware Limitations

The LiDAR-based safety guard has a known limitation: the RPLidar sensor cannot detect glass walls, mirrors, or transparent surfaces, and performs poorly on highly reflective or dark materials. This creates a systematic bias where the robot behaves more conservatively in standard lab environments than it would in real warehouse or retail settings where glass partitions are common. Similarly, the ArUco detection pipeline depends on consistent lighting — the system was validated only under lab fluorescent lighting and may degrade under direct sunlight or low-light conditions. From a Utilitarian standpoint, deploying this system in environments that differ significantly from the validation setting without additional testing would be irresponsible.

---

## 4. Custom Module Code Links

| Module | File | Primary Author |
|---|---|---|
| `board_pose_node.py` | [board_pose_node.py](https://github.com/Mobile-Robots-UGV/turtlebot4-sft-aruco-kf-pf-recovery/blob/main/board_pose_ros/board_pose_ros/board_pose_node.py) | Tatwik Meesala |
| `board_tracker_node.py` | [board_tracker_node.py](https://github.com/Mobile-Robots-UGV/turtlebot4-sft-aruco-kf-pf-recovery/blob/main/sft_hardware_tracker/sft_hardware_tracker/board_tracker_node.py) | Tatwik Meesala |
| `recovery_follower_node.py` | [recovery_follower_node.py](https://github.com/Mobile-Robots-UGV/turtlebot4-sft-aruco-kf-pf-recovery/blob/main/sft_hardware_tracker/sft_hardware_tracker/recovery_follower_node.py) | Lu Yan Tan |
| `leader_odom_tf_node.py` | [leader_odom_tf_node.py](https://github.com/Mobile-Robots-UGV/sim-to-real-integration/blob/main/sft_hardware_tracker/sft_hardware_tracker/leader_odom_tf_node.py) | Tatwik Meesala |
| `sft_hardware_recovery.launch.py` | [sft_hardware_recovery.launch.py](https://github.com/Mobile-Robots-UGV/turtlebot4-sft-aruco-kf-pf-recovery/blob/main/sft_hardware_tracker/launch/sft_hardware_recovery.launch.py) | Lu Yan Tan |

---

## 5. Individual Contribution & Audit Appendix

| Team Member | Primary Technical Role | Key Git Commits/PRs | Specific File(s) Authorship |
|---|---|---|---|
| Tatwik Meesala | Perception, detection pipeline, simulation integration | [7646c12](https://github.com/Mobile-Robots-UGV/turtlebot4-aruco-board-following-ros2/commit/7646c12) | `recovery_follower_node.py`, `sft_hardware_recovery.launch.py`, `leader_odom_tf_node.py` |
| Prajjwal | State estimation, simulation, prediction filters | [38f644f](https://github.com/Mobile-Robots-UGV/aruco-simulation-gazebo-rviz/commit/38f644f) | `sensor_fusion_ekf.py`, `sensor_fusion_ukf.py`, `sensor_fusion_pf.py`, `compare_filters.py`, `sft_hardware_recovery.yaml` |
| Lu Yan Tan | Coordination, control, SLAM integration, master launch | [a676342](https://github.com/Mobile-Robots-UGV/arucho-detection-goal-generator-coordinator/commit/a676342) | `board_pose_node.py`, `board_tracker_node.py`, `sft_turtlebot_slam.launch.py` |

---

## 6. Mid-Point to Final: What Changed

Between Milestone 2 and Milestone 3, the system was extended in the following ways:

- **PD angular controller** replaced the original P-only controller, adding a derivative term `kd_angular` to dampen overshoot and reduce oscillation during turning
- **Deadband on X and Z errors** was added to prevent continuous micro-corrections when the board is approximately centered
- **Subscriber queue depth reduced to 1** with Best Effort QoS across all nodes to eliminate stale data buildup that caused delayed reactions
- **OpenCV version compatibility** was added to handle both legacy (`detectMarkers`) and new (`ArucoDetector`) ArUco APIs across different Ubuntu/ROS2 installations
- **SLAM integration** via `namespace:=/robot_09` was validated on hardware, enabling real-time map building during following without any topic remapping
- **Single-command master launch file** was developed to start all nodes simultaneously with optional SLAM and RViz2 flags
- **Simulation pipeline** was integrated from teammate's repository, enabling staged testing in Gazebo with a teleop leader robot before hardware deployment