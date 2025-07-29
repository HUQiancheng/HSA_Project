# HSA Project - E-Skin Controlled Vehicle with SLAM

## Project Overview
E-Skin tactile sensor controlled mobile robot with IMU trajectory recording and SLAM capabilities.

### Hardware Configuration
- **Main Controller**: Raspberry Pi 5 (Debian Bookworm 64-bit)
- **Tactile Sensors**: 4× HEX-o-skin (ID 1-4: front/left/back/right)
- **IMU**: Adafruit BNO085 (9-DOF)
- **Distance Sensor**: Garmin LIDAR-Lite v4 (single-point)
- **Actuators**: 4× DC gear motors with dual L298N drivers
- **Microcontroller**: Arduino UNO

## Project Structure
```
HSA_Project/
├── docs/                    # Hardware documentation
├── ros2_slam_ws/           # ROS2 SLAM workspace (documentation snapshot)
├── test_mcu_ws/            # Arduino control code (PlatformIO)
├── test_skin_ws/           # ROS Noetic tactile sensor workspace
└── set_permissions.sh      # Port permission script
```

## Important Note on ROS2 SLAM Workspace
The ROS2 SLAM system was developed by Liangyu Chen directly on Raspberry Pi 5 hardware. This repository contains a documentation snapshot - the code cannot be executed without the complete ROS2 workspace structure and hardware setup that exists on the original Raspberry Pi system. External libraries and dependencies have been removed.

For detailed technical architecture and implementation details, see:
- `ros2_slam_ws/Readme.md`
- `ros2_slam_ws/src/basicsetup.md`

## Original Code Components

### ROS2 SLAM Module (`ros2_slam_ws/`)
Sophisticated robotic localization and SLAM system featuring:
- Multi-sensor fusion (wheel encoders, IMU, distance sensor)
- Extended Kalman Filter implementation
- SLAM adapted for single-point distance sensors
- Real-time trajectory recording and visualization

### Arduino Control Module (`test_mcu_ws/arduino_integrated/`)
- **Headers**:
  - `include/motor_control.h` - Motor control interface
  - `include/lidar_sensor.h` - Distance sensor interface
- **Source**:
  - `src/main.cpp` - Main control program
  - `src/motor_control.cpp` - Motor driver implementation
  - `src/lidar_sensor.cpp` - Distance sensor driver
- **Configuration**:
  - `platformio.ini` - PlatformIO project configuration

### ROS Tactile Sensor Module (`test_skin_ws/src/`)
- **skin_force_publisher package**:
  - `msg/FourCellForces.msg` - Custom message format
  - `src/force_publisher_node.cpp` - Force data processing node
  - `scripts/motor_control.py` - Tactile-based motor control
  - `launch/force_publisher.launch` - Package launcher
- **lidar_publisher package**:
  - `scripts/lidar_publisher_node.py` - Distance sensor ROS wrapper
  - `launch/lidar_publisher.launch` - Sensor launcher

## System Architecture
```
Sensors: HEX-o-skin×4 + IMU (BNO085) + Distance Sensor
    ↓
Processing: Raspberry Pi 5 (ROS/ROS2) + Arduino (low-level control)
    ↓
Actuators: 4×DC motors (L298N drivers)
```

## Quick Start

### 1. Environment Setup
```bash
# Set device permissions
./set_permissions.sh

# Build ROS workspace
cd test_skin_ws && catkin_make

# For ROS2 workspace setup, see ros2_slam_ws/src/basicsetup.md

# Upload Arduino firmware
cd test_mcu_ws/arduino_integrated
pio run -t upload
```

### 2. Run System
```bash
# Terminal 1: Tactile sensors
roslaunch skin_force_publisher force_publisher.launch

# Terminal 2: Distance sensor
roslaunch lidar_publisher lidar_publisher.launch

# Terminal 3: SLAM system (requires full ROS2 setup on Pi)
ros2 launch sensor_publisher slam_localization_launch.py
```

## Key Interfaces
- **ROS Topics**:
  - `/skin_forces` - Tactile force data (FourCellForces)
  - `/lidar_data` - Distance measurements
- **ROS2 Topics**:
  - `/imu/data_raw` - Raw IMU data
  - `/wheel_odom` - Wheel encoder odometry
  - `/scan` - LaserScan format distance data
  - `/odometry/odom_filtered` - Fused odometry estimate
  - `/trajectory` - Robot path history

## Development Guidelines
- Work in feature branches, merge via PR review
- Do not edit main branch directly
- ROS2 components require on-device development/testing

---
**HSA Project Team - 2024**