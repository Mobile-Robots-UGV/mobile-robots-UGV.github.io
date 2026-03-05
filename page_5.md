---
title: 5 Git Infrastructure & Timeline
layout: home
nav_order: 5
---


## 5. Git Infrastructure
### 5.1 Link to shared team repository
<https://github.com/Mobile-Robots-UGV/mobile-robots-UGV.github.io>
### 5.2 Git Submodule setup
This website is included in the main project repository as a Git submodule.

## Project Schedule(Extra)
### Gantt Chart Timeline (Weeks 7–16)

| Week | Task Category | Planned Work |
|------|---------------|--------------|
| **Week 7** | Hardware Integration | Verify TB4 base, networking, teleop mobility |
|  | Sensors | Bring up LiDAR + RGB‑D drivers; visualize raw topics |
|  | Controls & Autonomy | Validate `/cmd_vel`, odom, TF tree |
|  | Interface & Data | Basic RViz setup (robot model + LiDAR) |
|  | Milestone | Robot moves via teleop; all sensors online |

| **Week 8** | Hardware Integration | Mount camera; cable management |
|  | Sensors | Calibrate RGB‑D; verify depth stream |
|  | Controls & Autonomy | Configure EKF with IMU + odom |
|  | Interface & Data | RViz fused pose visualization |
|  | Milestone | Stable localization pipeline |

| **Week 9** | Hardware Integration | Long‑duration hardware test |
|  | Sensors | Collect rosbag for mapping |
|  | Controls & Autonomy | Bring up Nav2; point‑to‑point navigation |
|  | Interface & Data | Visualize map + path |
|  | Milestone | Robot navigates in simple map |

| **Week 10** | Hardware Integration | Validate bumper, cliff, wheel‑drop |
|  | Sensors | Ground‑plane removal + filtering |
|  | Controls & Autonomy | Tune local planner for narrow aisles |
|  | Interface & Data | Begin trajectory logging |
|  | Milestone | Safe navigation in semi‑cluttered space |

| **Week 11** | Hardware Integration | Integrate fiducial markers on OOI |
|  | Sensors | Implement AprilTag OOI detection |
|  | Controls & Autonomy | Implement follow behavior (distance + heading) |
|  | Interface & Data | Visualize OOI pose + follow distance |
|  | Milestone | Robot follows OOI in open space |

| **Week 12** | Hardware Integration | Stress‑test hardware during follow runs |
|  | Sensors | Add occlusion‑handling logic |
|  | Controls & Autonomy | Integrate follow + obstacle avoidance |
|  | Interface & Data | Structured evidence log (OOI + timestamps) |
|  | Milestone | Follow + avoid obstacles in cluttered space |

| **Week 13** | Hardware Integration | Validate E‑Stop + deadman switch |
|  | Sensors | Tune timeouts + failure responses |
|  | Controls & Autonomy | Implement fallback modes (search, stop, reacquire) |
|  | Interface & Data | Export map + trajectory + OOI log |
|  | Milestone | Full pipeline operational with safety guarantees |

| **Week 14** | Hardware Integration | Minor stabilization only |
|  | Sensors | Final tuning of filters + thresholds |
|  | Controls & Autonomy | Scenario tests: occlusions, tight aisles, dynamic agents |
|  | Interface & Data | Build dashboard/notebook for log review |
|  | Milestone | Meets baseline success criteria |

| **Week 15** | Hardware Integration | Freeze hardware configuration |
|  | Sensors | Regression tests |
|  | Controls & Autonomy | Freeze parameters; document configs |
|  | Interface & Data | Polish visualizations for demo/report |
|  | Milestone | System ready for final demo |

| **Week 16** | Hardware Integration | Demo prep: charging, spares, setup |
|  | Sensors | Quick health checks |
|  | Controls & Autonomy | Run final scripted demo |
|  | Interface & Data | Capture final logs, screenshots, videos |
|  | Milestone | Final demo completed |

