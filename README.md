# Autonomous Robot Navigation (GPS + LiDAR)

This project implements an autonomous mobile robot that navigates to GPS target points while avoiding obstacles using LiDAR.

The robot:
- Navigates using GPS coordinates
- Uses IMU for heading estimation
- Avoids obstacles using LiDAR
- Combines navigation and obstacle avoidance

## Team Members
- Jumbo Jaramilo Ivannova Michelle
- Li Xuetao

## System Components

- GPS (/gnss)
- IMU (/imu/mag)
- LiDAR (/scan)
- ROS (Robot Operating System)

## Algorithm

1. Get target GPS coordinate
2. Compute heading and distance
3. Read LiDAR data
4. Decision:
   - If obstacle detected → avoid
   - Else → move toward target

## How to Run

```bash
# Enable CAN
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

# Launch robot
roslaunch scout_bringup scout_minimal.launch

# Start LiDAR
roslaunch velodyne_pointcloud VLP16_points.launch

# Run program
python gpsfinalObs_3.1.py
