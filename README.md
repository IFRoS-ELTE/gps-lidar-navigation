# Autonomous Robot Navigation (GPS + LiDAR)

## Team Members

- Jumbo Jaramilo Ivannova Michelle
- Li Xuetao

## 1. Introduction

This project implements an autonomous mobile robot that navigates to GPS target points while avoiding obstacles using LiDAR.

The robot:
- Navigates using GPS coordinates
- Uses IMU for heading estimation
- Avoids obstacles using LiDAR
- Combines navigation and obstacle avoidance

Key Technologies
- ROS1 (Robot Operating System)
- Python (rospy)
- GPS (NavSatFix)
- IMU (MagneticField)
- LiDAR (LaserScan)

## 2. System Architecture

The system is composed of three main modules:

   1. GPS Navigation Module
      Computes distance to target using Haversine formula
      Calculates heading toward target
   2. IMU Orientation Module
      Estimates robot orientation (north angle)
      Used to align robot with target direction
   3. LiDAR Obstacle Avoidance Module
      Detects obstacles in front/left/right directions
      Overrides navigation when necessary

## 3. How to Run

```bash
# 1. Connect to Robot
ssh agilex@192.168.1.102

# 2. Enable CAN
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

# 3. Launch robot
roslaunch scout_bringup scout_minimal.launch

# 4. Start LiDAR
roslaunch velodyne_pointcloud VLP16_points.launch

# 5. Run program
python gpsfinalObs_3.1.py
```

## 4. Algorithm

Navigation Logic
- Get current GPS position
- Compute distance and heading to target
- Compare heading with current orientation
- Rotate until aligned
- Move forward

Obstacle Avoidance Logic
- Read LiDAR data
- Detect obstacles in front
- Choose direction (left/right)
- Override GPS control

State Switching
- Navigation Mode: No obstacle
- Avoidance Mode: Obstacle detected

## 5. Simulation & Real-World Performance

Simulation Tools:
During development, an online simulator, Robotics Learning Studio, was used to prototype and test obstacle avoidance behavior.

The simulation environment allowed rapid testing of:
- Basic obstacle avoidance strategies
- Turning logic (left/right decision)
- Forward motion control

This significantly accelerated early-stage development before deploying to the real robot.

Simulation vs Real-World Differences

Although the simulation provided useful initial validation, several key differences were observed when transitioning to the real robot:

1. GPS Dependency
In simulation, robot positioning is precise and noise-free
In real-world scenarios, GPS introduces:
- Noise
- Drift
- Inconsistent target coordinates across sessions
Robot may stop too early or overshoot targets.

2. Obstacle Representation
- Simulation uses idealized obstacle shapes
- Real-world obstacles vary significantly in size and geometry

Observed Behavior:
|Obstacle Type  | Performance          |
|---------------|----------------------|
| Thin objects (e.g., poles)  | Successfully avoided    |
| Wide objects (e.g., trees)  | Avoidance failed or inefficient   |

## 6. Results

The robot successfully:

- Navigates to multiple GPS targets
- Avoids obstacles in real time
- Switches between navigation and avoidance modes
- Maintains stable motion without oscillation

## 7. Troubleshooting and Practical Issues

1. Robot Oscillates (Left-Right Movement)
- Problem:
   Robot moves forward but swings left and right.
- Cause:
   Simultaneous turning and forward motion.
- Solution:
   Separate turning and forward movement.
   Only move forward when aligned.

2. Robot Spins in Place
- Cause:
   Heading threshold too strict.
- Solution:
   Increase devlimit (e.g., 20 → 30 degrees)

3. /gnss Topic Missing

-Cause: GPS driver not running
-Solution:
   rostopic list
   Ensure GPS node is active.

4. Robot Skips Target Points
- Cause: Target threshold too large
- Solution:
   Reduce targetstopdistance (e.g., 3 → 2 meters)
5. Robot Gets Stuck in Avoidance
- Cause: Obstacle always detected
- Solution:
   Reduce obstacle threshold
   Allow slow forward movement

## 8. Conclusion

This project demonstrates a complete autonomous navigation pipeline combining:

Global positioning (GPS)
Local sensing (LiDAR)
Orientation estimation (IMU)

The system successfully balances global navigation and local obstacle avoidance, forming a foundation for more advanced autonomous robotic systems.

## 9. Future Work
   Add SLAM for indoor navigation
   Implement path planning (A*, DWA)
   Improve control using PID
   Integrate full ROS Navigation Stack
   Add camera-based perception
