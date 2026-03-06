# SmartFollower & Tracker (SFT) — Documentation Website

This repository hosts the documentation site for **SmartFollower & Tracker (SFT)**: an indoor **TurtleBot 4–based** autonomous system that detects, tracks, and safely follows a designated **Object of Interest (OOI)** in a dynamic warehouse/packaging environment, while avoiding obstacles and producing a **2D reconstruction** (map + trajectory + time-stamped evidence) for anomaly investigation and audit.

## What SFT Does

SFT is designed for indoor warehouse/factory logistics zones with dynamic obstacles, frequent occlusions, and shared human–robot space constraints. The system aims to:
- Acquire an OOI using onboard sensors
- Track the OOI robustly (including temporary occlusions)
- Follow while maintaining safety constraints (distance/speed/collision avoidance)
- Reacquire the OOI after loss or transition to safe fallback behavior
- Export a 2D occupancy map, robot trajectory, and time-stamped OOI observations.

## Documentation Structure

The site content is organized into the following pages (see the navigation on the deployed site):
- **Home (Mission & Scope):** environment, constraints, scope, and measurable acceptance criteria 
- **Technical Specifications:** TurtleBot 4 platform, kinematics, baseline sensors, and safety/hazard sensing stack
- **Architecture:** two-phase workflow (pre-mapping + mission), data-flow diagram, state machine, and module table (Perception → Estimation → Planning → Actuation)
- **Safety & Operational Protocol:** sensor-based safety layers, physical constraints, deadman switch, timeouts, and E-stop conditions
- **Git Infrastructure:** shared repository link and submodule note
- **Project Schedule:** week-by-week plan and milestones

