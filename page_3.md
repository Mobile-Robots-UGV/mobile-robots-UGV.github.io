---
title: 3 Architecture
layout: home
nav_order: 3
---


## 3. High-Level System Architecture

This autonomous mobile robot system operates in two phases: **(A) Pre-Mapping** to build a static 2D occupancy grid via teleoperated SLAM, and **(B) Mission** to autonomously follow a moving ArUco marker-bearing target while avoiding unknown obstacles. The architecture follows the **Perception → Estimation → Planning → Actuation** convention.

### 3.1 System Data Flow Diagrams

#### 3.1.1 Mission Mode Data Flow (Perception → Estimation → Planning → Actuation)

```mermaid
graph LR
    subgraph Perception["🔹 PERCEPTION"]
        Lidar["LiDAR Driver<br/>/scan"]
        Camera["Camera Driver<br/>/camera/image_raw"]
        ArUco["ArUco Detector<br/>/detected_aruco_pose<br/>(Custom)"]
    end

    subgraph Estimation["🔹 ESTIMATION"]
        AMCL["AMCL<br/>/amcl_pose<br/>(Library)"]
        Costmap["Local Costmap<br/>obstacle layer<br/>(Library)"]
        TF["TF2 System<br/>frame transforms<br/>(Library)"]
        GoalGen["Goal Generator<br/>/target_goal_pose<br/>(Custom)"]
        Coord["Target Coordinator<br/>state machine<br/>(Custom)"]
    end

    subgraph Planning["🔹 PLANNING & CONTROL"]
        Nav2["Nav2 Stack<br/>planner + controller<br/>(Library)"]
    end

    subgraph Actuation["🔹 ACTUATION"]
        Motor["Base Controller<br/>motor commands<br/>(Library)"]
    end

    Lidar --> AMCL
    Lidar --> Costmap
    Camera --> ArUco
    ArUco --> GoalGen
    ArUco --> Coord
    AMCL --> TF
    TF --> GoalGen
    GoalGen --> Coord
    Coord --> Nav2
    Costmap --> Nav2
    Nav2 --> Motor

    style Perception fill:#a8e6cf
    style Estimation fill:#ffe66d
    style Planning fill:#ff6b6b
    style Actuation fill:#4ecdc4
```

#### 3.1.2 State Machine: Target Follow + Loss Recovery

```mermaid
stateDiagram-v2
    [*] --> FOLLOW: Mission start / target visible
    FOLLOW --> LOST: 5+ frame misses AND LOS blocked
    LOST --> FOLLOW: ArUco re-detected
    LOST --> SEARCH: Timeout expires
    SEARCH --> FOLLOW: ArUco detected
    SEARCH --> SEARCH: Rotating/scanning

    note right of FOLLOW
        Dynamic goal updates.
        Nav2 tracking path.
    end note

    note right of LOST
        Goal frozen at last pose.
        Nav2 replans around obstacles.
    end note

    note right of SEARCH
        Rotational sweep for
        target re-acquisition.
    end note
```

---

### 3.2 Module Declaration Table

| **Module Name** | **Type** | **ROS 2 Package** | **Inputs** | **Outputs** |
|---|---|---|---|---|
| **LiDAR Driver** | Library | Driver-specific | Hardware | `/scan` (sensor_msgs/LaserScan) |
| **Wheel Odometry** | Library | Base firmware | Motor encoders | `/odom`, `/tf` (odom→base_link) |
| **Camera Driver** | Library | usb_cam / cv_camera | Hardware | `/camera/image_raw` (sensor_msgs/Image) |
| **SLAM Toolbox** | Library | slam_toolbox | `/scan`, `/tf`, `/odom` | `/map`, `/tf` (map↔odom) |
| **Map Server** | Library | nav2_map_server | Disk file | `/map` (nav_msgs/OccupancyGrid) |
| **AMCL** | Library | nav2_amcl | `/scan`, `/map`, `/tf` | `/amcl_pose`, `/tf` (map→odom) |
| **Local Costmap** | Library | nav2_costmap_2d | `/scan`, `/tf`, `/amcl_pose` | Costmap layers (obstacle) |
| **Nav2 Planner** | Library | nav2_navfn_planner | `/map`, `/costmap`, `/tf` | `/plan` (nav_msgs/Path) |
| **Nav2 Controller** | Library | nav2_dwb_controller | `/costmap`, `/tf`, `/path` | `/cmd_vel` (geometry_msgs/Twist) |
| **ArUco Detector** | Custom | Custom (this project) | `/camera/image_raw` | `/detected_aruco_pose` (geometry_msgs/PoseStamped) |
| **Goal Generator** | Custom | Custom (this project) | `/detected_aruco_pose`, `/tf` | `/target_goal_pose` (geometry_msgs/PoseStamped) |
| **Target Coordinator** | Custom | Custom (this project) | `/detected_aruco_pose`, `/amcl_pose`, `/scan`, costmap | `/navigate_to_pose` action goals |
| **TF2 Broadcaster** | Library | tf2_ros | Sensor poses, odometry | `/tf`, `/tf_static` |
| **Base Controller** | Library | Base firmware | `/cmd_vel` | Motor PWM commands |

---

### 3.3 Module Intent Writeups

#### Library Modules

- **LiDAR Driver:** Publishes 2D laser scans at 10–25 Hz. Tune: scan rate, max range, angular resolution (typical: 25 Hz, 8 m range).

- **Wheel Odometry & Base Controller:** Encoder-based odometry at ~50 Hz; controller translates `/cmd_vel` to motor PWM. Tune: wheel radius, track width, drift compensation (<30 cm over 30 sec).

- **Camera Driver:** Publishes RGB frames at 15–30 Hz with precomputed intrinsic calibration. Tune: resolution, frame rate, distortion coefficients.

- **SLAM Toolbox:** Standard ROS 2 SLAM front-end for 2D LiDAR. Builds occupancy grid via scan matching and loop closure. Tune: loop closure threshold, min travel distance, occupancy threshold.

- **Map Server:** Loads precomputed occupancy grid from disk for mission phase.

- **AMCL:** Localizes against static map by matching LiDAR scans; corrects odometry drift. Tune: particle count (50–500), scan matching model, update rates.

- **Local Costmap:** Maintains rolling 5×5 m window, marks pre-mapped and dynamic obstacles. Tune: decay rate, inflation radius, update frequency.

- **Nav2 Planner & Controller:** Global planner (NavFn) + local controller (DWB). Tune planner: potential scale, lethal cost threshold. Tune controller: max velocity, acceleration limits, lookahead distance.

- **TF2 System:** Manages coordinate frame transformations (map ← odom ← base_link). Configure: static transforms (sensor offsets), dynamic transforms (AMCL, odometry).

- **Base Controller:** Translates motor commands from Nav2.

---

#### Custom Modules

**ArUco Detector**
Implements real-time visual marker detection using OpenCV. Algorithm: (1) subscribe to `/camera/image_raw` at camera frame rate; (2) apply adaptive thresholding to detect candidate marker corners; (3) validate corner patterns against ArUco dictionary (DICT_5X5_100); (4) reject false positives using confidence threshold; (5) solve PnP with known marker size and camera intrinsics to compute 3D pose in camera frame; (6) publish `geometry_msgs/PoseStamped` to `/detected_aruco_pose`. **Success criteria:** detects target marker at 0.5–3 m with <5% pose error; tolerates 1–2 frame intermittent occlusions; rejects false positives >95%.

**Goal Generator from ArUco**
Transforms target pose from camera frame to map frame and computes reachable goal. Algorithm: (1) subscribe to `/detected_aruco_pose`; (2) query TF2 for camera→base_link→odom→map chain; (3) transform pose to map frame; (4) offset goal 0.5 m behind target (approach direction); (5) apply moving-average temporal smoothing over last N detections; (6) publish `/target_goal_pose` at ~10 Hz. Ensures goal is always reachable by the robot. **Success criteria:** <0.1 m transform error; goal always reachable; temporal jitter <0.2 m s.d.

**Target Follow + Loss Recovery Coordinator**
Implements state machine (FOLLOW → LOST → SEARCH) to handle target occlusion and recovery. Algorithm: **FOLLOW state:** continuously query `/detected_aruco_pose` and Goal Generator's `/target_goal_pose`; send goals to Nav2 at 5–10 Hz. **Transition to LOST:** detect ≥5 consecutive frame misses AND costmap raytrace confirms line-of-sight blockage; freeze goal at last-known target; start loss timer (2–5 sec). **LOST state:** goal is static; monitor for re-detection or clearing. **Transition to SEARCH:** loss timeout expires and ArUco still absent. **SEARCH state:** execute rotational sweep or frontier-based movement for 15–30 sec; on re-detection, immediately transition to FOLLOW. **Success criteria:** detect occlusion within 0.5 sec; re-acquire target within 10 sec; recover 90%+ of loss events.

---

### 3.4 Terminology Reference

| Term | Definition |
|---|---|
| **Pre-mapping** | Phase 1: teleoperated SLAM exploration to build offline 2D occupancy grid. |
| **Mission** | Phase 2: autonomous target-following with AMCL localization. |
| **Line of Sight (LOS)** | Direct unobstructed visual path from camera to ArUco marker. |
| **Occlusion-induced loss** | Target loss caused by obstacle blocking LOS. |
| **Unknown obstacle** | Real-time obstacle not in pre-mapped grid; detected by LiDAR. |
| **Local costmap** | Rolling 5×5 m costmap updated from LiDAR; used by Nav2 controller. |
| **Recovery behavior** | Fallback strategy (e.g., search rotation) for target re-acquisition. |

