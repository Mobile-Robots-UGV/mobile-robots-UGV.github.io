---
title: 2 Specifications
layout: home
nav_order: 2
---

## 2. Technical Specifications

### 2.1 Robot Platform
**Hardware platform:** TurtleBot 4 (Create 3 differential-drive base) with integrated sensing stack.
- **Create 3 base:** wheel encoders, IMU, hazard sensors (bump, cliff, wheel-drop, etc.)
- **2D LiDAR:** for obstacle detection and 2D mapping inputs
- **RGB-D camera:** for OOI detection/tracking and depth-based range estimation
- **Compute:** onboard (Raspberry Pi / platform default), with optional offboard workstation if explicitly documented

**Simulation environment (for early integration):**
- TurtleBot 4 simulation in Gazebo/Streamlit (used for early pipeline verification and regression tests)

### 2.2 Kinematic Model
**Declared model:** Differential Drive (unicycle-equivalent planar model)

Let robot pose be **x = [x, y, θ]ᵀ** and control inputs be **u = [v, ω]ᵀ**:
- **ẋ = v cos(θ)**
- **ẏ = v sin(θ)**
- **θ̇ = ω**

Wheel mapping (for reference; r = wheel radius, L = wheel separation):
- **v = (r/2)(ω_R + ω_L)**
- **ω = (r/L)(ω_R − ω_L)**

### 2.3 Perception Stack (Sensors Used)
**Required baseline sensors:**
- **2D LiDAR:** obstacle geometry, local collision checking, 2D mapping support
- **RGB-D camera:** OOI detection/tracking + depth-based range to target
- **IMU:** angular rate/orientation aiding for stable heading estimation
- **Wheel encoders/odometry:** incremental motion estimation for navigation and tracking stabilization

**Safety and hazard sensing:**
- **Bumper switches:** contact detection → immediate stop / back-off logic
- **Cliff sensors:** edge detection near docks/ramps → stop and retreat
- **Wheel-drop / stall / slip / kidnap detection:** triggers safe halt and recovery protocol

### 2.4 Additional Sensors / Safety Additions

- **Fiducial markers (AprilTag) on OOI:**
  - Justification: robust ID + relative pose for reliable follow control under occlusion/lighting variability [1].

  - Justification: non-line-of-sight ranging cues to improve reacquisition in shelving occlusions.
- **Short-range ToF/proximity sensors around blind spots (optional):**
  - Justification: mitigates single-plane LiDAR limitations (low/high obstacles, near-base blind regions).
- **Operator annunciation (LED/beeper) (operational safety enhancement):**
  - Justification: improves human awareness when robot enters follow/chase modes.
