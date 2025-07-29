# Test Skin Workspace

## Overview
ROS Noetic workspace containing tactile sensor and distance measurement packages for the HSA vehicle project.

## Workspace Structure
```
test_skin_ws/
├── launch/
│   └── configs/
│       └── default.xml              # E-skin configuration
├── src/
│   ├── lidar_publisher/            # Distance sensor ROS wrapper
│   └── skin_force_publisher/       # E-skin tactile interface
```

## Packages

### skin_force_publisher
Interfaces with 4 HEX-o-skin tactile units (IDs 1-4: front/left/back/right) using TUM ICS skin driver.

**Components**:
- `src/force_publisher_node.cpp` - Reads 3 force sensors per unit, publishes averaged values
- `msg/FourCellForces.msg` - Custom message: `float64[4] forces`
- `scripts/motor_control.py` - Translates tactile forces to motor commands
- `launch/force_publisher.launch` - Integrated launch (skin driver + force publisher + motor control)

**Published Topic**: `/skin_forces` at 30Hz

### lidar_publisher  
Wraps Arduino serial distance data in LaserScan format for SLAM compatibility.

**Components**:
- `scripts/lidar_publisher_node.py` - Serial reader, converts "L:xxx" format to LaserScan
- `launch/lidar_publisher.launch` - Node launcher

**Configuration**:
- Serial: `/dev/ttyACM0` at 9600 baud (Arduino firmware in `test_mcu_ws/`)
- 10cm offset correction applied
- Single-point sensor adapted to LaserScan message (angle_min = angle_max = 0)

**Published Topic**: `/scan` at 20Hz

## Usage
```bash
# E-skin system with motor control
roslaunch skin_force_publisher force_publisher.launch

# Distance sensor
roslaunch lidar_publisher lidar_publisher.launch
```

## Docker Development
Uses TUM ICS ROS skin image. VS Code Dev Container configuration included.