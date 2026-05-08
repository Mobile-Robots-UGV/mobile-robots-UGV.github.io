---
title: 6 Project Schedule
layout: home
nav_order: 6
---

### Gantt Chart Timeline (Weeks 7–16)

Status legend: ✅ Completed | 🔄 In Progress | 🕒 Planned

| Week | Task Category | Planned Work | Status |
|------|---------------|--------------|--------|
| **Week 7** | Hardware Integration | Verify TB4 base, networking, teleop mobility | ✅ |
|  | Sensors | Bring up LiDAR + RGB-D drivers; visualize raw topics | ✅ |
|  | Controls & Autonomy | Validate `/cmd_vel`, odom, TF tree | ✅ |
|  | Interface & Data | Basic RViz setup (robot model + LiDAR) | ✅ |
|  | Milestone | Robot moves via teleop; all sensors online | ✅ |
| **Week 8** | Hardware Integration | Mount camera; cable management | ✅ |
|  | Sensors | Calibrate OAK-D; verify compressed RGB stream | ✅ |
|  | Controls & Autonomy | Configure WiFi-based OAK-D topic subscription | ✅ |
|  | Interface & Data | RViz ArUco debug image visualization | ✅ |
|  | Milestone | Stable camera pipeline over ROS 2 WiFi | ✅ |
| **Week 9** | Hardware Integration | Long-duration hardware test | ✅ |
|  | Sensors | ArUco board detection and solvePnP pose estimation | ✅ |
|  | Controls & Autonomy | Camera frame coordinate convention established | ✅ |
|  | Interface & Data | Visualize board pose + TF in RViz2 | ✅ |
|  | Milestone | Board pose estimated in real time on hardware | ✅ |
| **Week 10** | Hardware Integration | Hardware test with TurtleBot 4 moving | ✅ |
|  | Sensors | EMA smoothing + board visibility reporting | ✅ |
|  | Controls & Autonomy | Direct velocity control via `robot_09/cmd_vel` | ✅ |
|  | Interface & Data | RViz2 markers for board center, orientation, distance | ✅ |
|  | Milestone | Robot receives and responds to velocity commands | ✅ |
| **Week 11** | Hardware Integration | End-to-end hardware pipeline test | ✅ |
|  | Sensors | Compressed image decoding; 4-case quaternion conversion | ✅ |
|  | Controls & Autonomy | Proportional follow controller with deadband | ✅ |
|  | Interface & Data | TF broadcast `camera_frame` → `board_frame` | ✅ |
|  | Milestone | Robot follows ArUco board in open space | ✅ |
| **Week 12** | Hardware Integration | Stress-test follow runs on TurtleBot 4 | ✅ |
|  | Sensors | Pose age timeout; board loss detection | ✅ |
|  | Controls & Autonomy | FOLLOW / LOST / SEARCH state machine | ✅ |
|  | Interface & Data | Coordinator state markers in RViz2 | ✅ |
|  | Milestone | Follow + recovery pipeline working on hardware | ✅ |
| **Week 13** | Hardware Integration | Validate backward motion and clamping behavior | ✅ |
|  | Sensors | EKF / UKF / Particle Filter implemented in simulation | ✅ |
|  | Controls & Autonomy | goal_generator placeholder integrated; pipeline end-to-end | ✅ |
|  | Interface & Data | Filter comparison visualization; Milestone 2 report published | ✅ |
|  | Milestone | Milestone 2 submitted; full pipeline operational on hardware | ✅ |
| **Week 14** | Hardware Integration | Minor stabilization and parameter tuning | ✅ |
|  | Sensors | KF/PF tracker integrated; measured/predicted/lost states validated | ✅ |
|  | Controls & Autonomy | PD angular controller + deadband added; oscillation eliminated | ✅ |
|  | Interface & Data | Debug image pipeline; low-latency RViz2 camera feed | ✅ |
|  | Milestone | Prediction-based following working end-to-end on hardware | ✅ |
| **Week 15** | Hardware Integration | Hardware configuration frozen; COLCON_IGNORE structure set up | ✅ |
|  | Sensors | SLAM integration validated with `namespace:=/robot_09` | ✅ |
|  | Controls & Autonomy | Parameters frozen; OpenCV version compatibility fix applied | ✅ |
|  | Interface & Data | Master launch file; RViz2 config saved; website updated | ✅ |
|  | Milestone | System ready for final demo | ✅ |
| **Week 16** | Hardware Integration | Demo prep: charging, spares, setup | ✅ |
|  | Sensors | Health checks; board detection verified | ✅ |
|  | Controls & Autonomy | Final scripted demo executed | ✅ |
|  | Interface & Data | Milestone 3 report published; demo videos captured | ✅ |
|  | Milestone | Final demo completed | ✅ |