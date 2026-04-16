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
| **Week 14** | Hardware Integration | Minor stabilization and parameter tuning | 🔄 |
|  | Sensors | Replace goal_generator placeholder with EKF/UKF prediction | 🔄 |
|  | Controls & Autonomy | Scenario tests: occlusions, target acceleration, recovery | 🔄 |
|  | Interface & Data | Log follow performance; capture demo videos | 🔄 |
|  | Milestone | Prediction-based following working end-to-end | 🔄 |
| **Week 15** | Hardware Integration | Freeze hardware configuration | 🕒 |
|  | Sensors | Regression tests with full prediction pipeline | 🕒 |
|  | Controls & Autonomy | Freeze parameters; document final configs | 🕒 |
|  | Interface & Data | Polish visualizations and website for final demo | 🕒 |
|  | Milestone | System ready for final demo | 🕒 |
| **Week 16** | Hardware Integration | Demo prep: charging, spares, setup | 🕒 |
|  | Sensors | Quick health checks | 🕒 |
|  | Controls & Autonomy | Run final scripted demo | 🕒 |
|  | Interface & Data | Capture final logs, screenshots, videos | 🕒 |
|  | Milestone | Final demo completed | 🕒 |