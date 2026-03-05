---
title: 4 Safety & Operational Protocol
layout: home
nav_order: 4
---



## 4. Safety & Operational Protocol
### 4.1 Sensors-Based Safety Layers
The Turtlebot 4 uses multiple redundant sensing layers to maintain safe operation in dynamic warehouse environments.
- **Lidar Snesor:**
  Allow the turtlebot to react when an obstacle is detected within a specific range
- **Bump Sensor:**
  Pause movement when hitting obstacles after Lidar missed the object detection
- **Cliff Sensor:**
  Prevent robot from falling down at the edge of a surface
- **Wheel-drop Sensor:**
  Detect whether the turtlebot if lifted and stop the wheel from operating
- **IMU:**
  Monitors abnormal tilt, acceleration, or impacts to handle unexpected motion by slowing down/stop.

### 4.2 Pysical Contraints
Physical contraints describe the non-negotiable real-world limits that the Turtlebots must abide to ensure safe and predictable behavior.
- **Turtlebot Dimensions:**
  The body frame size of the Turtlebot 4 and how the sensors are mounted have defines the minimum aisle width the robot can safely navigate without scraping pallets or shelving.
- **Speed Limits:**
  The robot must cap linear and angular velocity to remain safe around obstacles. The speed limits is directly tied to how fast the sensors can update and how far the robot need to stop.

### 4.3 Software Deadman Switch
The Deadman Switch ensures the robot never moves when safety conditions are not met. When the Deadman Switch is triggered, the robot immediately halts all motion, cancels navigation goals, and requires manual operator reset.
- **Message Timer:**
  Missing /cmd_vel messages within 500ms halt the operation.
- **Missing OOI:**
  OOI is not detected within a specific amount of time
- **Battery Life Prediction:**
  End the program when robots is close to running out of battery to ensure robot stay in a relatively safe position

### 4.4 Timeout Logic
Timeouts Logic decides when the robot must stop, slow down, or switch into a different mode because a critical subsystem no longer work reliably.
- **Sensor Timeouts:**
  The turtlebot has to stop moving when Lidar stops publishing messages.
- **Tracking Timeouts:**
  If OOI is lost for a threshold time, enters "search" or "safe fallback" mode.
- **Planning Timeouts:**
  Each operation is time-limited and once the time is up, the turtlebot should end the operation and return the final position of the OOI.
- **Slow Update Speed:**
  When the message/sensor update rate is slow, reduce robot speed.

### 4.5 Emergency Step (E-Stop Condition)
The E-Stop defines the highest-priority safety mechanism. It forces irreversible motor shutdown until a human operator manually resets the system.
**The E-Stop happens when:**
  - Wheel-drop triggered.
  - Cliff sensor triggered.
  - Bumper triggered.
  - IMU tilted beyond safe limits.
  - Extremely low battery level detected.
  - Manual stop triggered by operators.

**E-Stop behavior:**
  - All motor commands are disabled.
  - Navigation goals are canceled.
  - No autonomous recovery is allowed(Needs a manual reset).





