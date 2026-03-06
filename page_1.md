---
title: 1 Proposal & Specifications
layout: home
nav_order: 1
---

# SmartFollower & Tracker (SFT) for Warehouse Anomaly Investigation
**Mission Statement:** Design and implement an indoor TurtleBot 4–based autonomous system that can detect, track, and safely follow a designated Object of Interest (OOI) through a dynamic warehouse/packaging environment, while circumventing obstacles and producing a 2D reconstruction (map + trajectory + time-stamped evidence) sufficient for anomaly investigation and audit.

---

## 1. Mission Statement & Scope
### 1.1 Operational Motivation
Warehouses and factory floors routinely experience exception events (e.g., missing package in a tote/pallet, misrouted items, damaged cartons, unexpected obstruction, suspected tampering, or anomalies inside loading bays/trucks). These events are labor-intensive to diagnose and often occur in dynamic, cluttered spaces. The SmartFollower & Tracker (SFT) robot provides a mobile “investigation and tracking” capability: it can follow an OOI (e.g., a tagged tote/cart/person), navigate around obstacles, and generate a 2D reconstruction with evidence logs to support rapid verification and root-cause analysis.

### 1.2 Target Environment
**Primary environment:** Indoor warehouse / factory logistics zones (no GPS), including:
- **Aisles & staging areas:** narrow aisles, pallet stacks, shelving occlusions
- **Packaging zones:** dense clutter, human traffic, carts, reflective floors
- **Loading-bay edges + truck interiors (optional test zone):** illumination changes, confined geometry, ramps/thresholds

**Key constraints:**
- Dynamic obstacles (workers, carts, pallets)
- Dynamic targets (humans, carts, tote)
- Frequent occlusions and lighting variability
- Safety requirements for shared human-robot space
- Narrow spaces for robot to pass through

### 1.3 Primary Problem Statement
Given a dynamic indoor warehouse environment without GPS, develop a mobile robot system that:
1. Acquires an OOI using onboard sensors
2. Tracks the OOI robustly (including temporary occlusions)
3. Follows/chases while maintaining safety constraints (distance, speed limits, collision avoidance)
4. Reacquires the OOI after loss or transitions to safe fallback behavior
5. Produces a 2D map + trajectory + time-stamped observations usable for anomaly investigation

### 1.4 Scope

#### In-Scope
- Single-robot autonomy on TurtleBot 4 (hardware-first, with simulation for early tests)
- OOI detection and tracking (baseline + stretch)
- Dynamic objects(constantly moving) & Static objects tracking
- Smart following control (distance/heading regulation)
- Obstacle circumvention (local reactive + optional global planning)
- 2D reconstruction (occupancy grid + path + evidence logs)

#### Out-of-Scope
- Safety-certified industrial compliance (we follow engineering best practices but do not claim certification)
- Full enterprise WMS/ERP integration
- Full 3D reconstruction/dense 3D mapping as a requirement
- Multi-robot swarming (listed only as future extension)

### 1.5 Success State (Measurable Acceptance Criteria)
**Baseline success criteria (must all be met in representative indoor test runs):**
1. **OOI acquisition:** detect OOI within ≤ 3 s after entering sensor FOV
2. **Tracking continuity:** maintain track ≥ 90% over a 5-minute run with ≥ 3 occlusion events
3. **Reacquisition:** reacquire within ≤ 5 s after temporary loss, else trigger safe fallback
4. **Following performance:** maintain follow distance 1.0 m ± 0.3 m (configurable)
5. **Safety outcome:** zero collisions; robot slows/stops and routes around obstacles within a defined safety radius
6. **2D reconstruction output:** export (i) occupancy map and (ii) trajectory + time-stamped OOI detection log

**Stretch goals:**
- Semantic OOI tracking without fiducials (e.g., person/tote/box) using MOT + depth fusion
- Operation in loading bay / short truck interior segments with lighting changes
- Multi-robot extension (swarm) as future work only

### 1.6 Assumptions & Constraints
- Flat indoor ground; ramps/thresholds allowed only within platform capability
- Network may be intermittent; autonomy must degrade safely
- Embedded compute limits may constrain perception throughput; document any offboard compute if used


---


