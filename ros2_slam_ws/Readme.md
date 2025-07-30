# Robotic Localization and SLAM System

## Overview

ROS2 SLAM system for Raspberry Pi 5 using wheel encoders, IMU, and single-point distance sensor. Enables mapping and localization with minimal hardware through software innovation.

**Note**: This workspace runs on Raspberry Pi 5. Code provided for documentation purposes.

## System Architecture

**Sensor Layer**: Hardware interfaces (encoders, IMU, distance sensor)
**Fusion Layer**: Extended Kalman Filter combining sensor data  
**SLAM Layer**: Map building from sparse sensor data
**Visualization**: Path tracking and RViz integration

## Core Components

### encoder_node1.py - Wheel Odometry
- GPIO interrupt-based encoder counting
- Differential drive kinematics: `linear_vel = (left + right)/2`, `angular_vel = (right - left)/wheelbase`
- Dynamic covariance: High confidence when stationary, lower when moving
- Publishes to `/wheel_odom`

### imu_node.py - BNO085 Interface  
- I2C communication with BNO085 sensor
- 20Hz publication of quaternion orientation, angular velocity, linear acceleration
- Publishes raw data to `/imu/data_raw` for external filtering

### lidar_node.py - Virtual 360° Scanner
**Key Innovation**: Converts single-point distance sensor to SLAM-compatible scan data
- Maps distance readings to 72 angular bins (5° resolution) based on robot heading
- Uses odometry feedback to determine current angle: `angle_index = int(robot_yaw/(2π) * 72)`
- Smooths neighboring angles with noise-augmented data
- Publishes LaserScan to `/scan`

### path_publisher.py - Trajectory Recording
- Subscribes to `/odometry/odom_filtered`
- Accumulates robot positions into Path message
- Publishes trajectory to `/trajectory` for RViz visualization

### simple_map_publisher.py - Test Map
- Publishes static 10×10 occupancy grid to `/test_map`
- Used for debugging visualization without SLAM

## Configuration Files

### ekf.yaml - Sensor Fusion Parameters
- Configures which sensor dimensions feed into EKF
- `odom0_config`: Uses x,y position, yaw orientation, x,y velocity from encoders
- `imu0_config`: Uses angular velocity and linear acceleration from IMU
- Defines process/measurement noise covariance matrices

### slam.yaml - SLAM Parameters
- Modified for sparse single-point sensor data
- Reduced matching thresholds: `minimum_travel_distance: 0.02m`
- Lowered response requirements for scan matching
- Optimized for limited sensor data vs traditional 360° scanners

### slam_localization_launch.py - System Orchestration
Launches complete system:
1. Sensor nodes (IMU, encoders, distance sensor)
2. Static transforms (sensor→base_link relationships)  
3. IMU complementary filter
4. EKF sensor fusion
5. SLAM algorithm
6. Path publisher

## Data Flow

```
Hardware → Sensor Nodes → Processing → Fusion → SLAM
Encoders → encoder_node1 → /wheel_odom → EKF → slam_toolbox
BNO085 → imu_node → /imu/data_raw → filter → /imu → EKF ↗  
Distance → lidar_node → /scan (virtual 360°) → slam_toolbox ↗
```

## Technical Achievements

- **Single-point SLAM**: Adapted traditional SLAM for sparse sensor data
- **Virtual scanning**: Created 360° scan data from single distance sensor
- **Smart covariance**: Motion-dependent uncertainty modeling
- **ARM optimization**: Successfully compiled complex packages on Raspberry Pi 5

## Hardware Requirements

- Raspberry Pi 5
- BNO085 IMU (I2C)
- Wheel encoders (GPIO pins 17, 27)
- Serial distance sensor (/dev/ttyACM0)
- Differential drive robot platform