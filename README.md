# HSA Project - E-Skin Controlled Mobile Robot Platform

## Project Overview
This project implements a mobile robot controlled through tactile interaction using TUM ICS electronic skin (e-skin) sensors. The system allows users to guide the robot by touching different sensors, with movement speed proportional to applied force. Additionally, the platform includes IMU trajectory recording and LiDAR mapping capabilities.

## System Architecture

### Dual-Environment Design
The project uses two separate ROS environments due to hardware access constraints:

1. **Docker Container Environment** (this repository)
   - Runs ROS Noetic with TUM ICS e-skin drivers
   - Handles e-skin data processing and motor control
   - Relays LiDAR data between Arduino and ROS2

2. **Raspberry Pi Native Environment** (separate installation)
   - Runs ROS2 for SLAM functionality
   - Direct GPIO access for IMU and wheel encoders
   - Performs sensor fusion and mapping

### Data Flow
```
Docker Container (ROS1):
E-Skin → skin_force_publisher → motor_control.py → Arduino → Motors
                                                      ↓
                                                   LiDAR data
                                                      ↓
                              lidar_publisher ← ─ ─ ─ ┘
                                    ↓
                              ros_bridge
                                    ↓
Raspberry Pi (ROS2):
IMU + Encoders → sensor_publisher → SLAM (trajectory + map)
```

## Hardware Components

### Sensors
- **4× HEX-o-skin Units**: Tactile force sensors
  - ID 1: Front position
  - ID 2: Left position  
  - ID 3: Back position
  - ID 4: Right position
- **Garmin LIDAR-Lite v4**: Single-point distance sensor
- **Adafruit BNO085**: 9-DOF IMU for orientation/acceleration
- **Wheel Encoders**: Odometry measurement

### Actuators
- **4× DC Gear Motors**: 2 left side, 2 right side
- **2× L298N Motor Drivers**: One per side

### Control
- **Raspberry Pi 5**: Main computer (Debian Bookworm 64-bit)
- **Arduino UNO**: Low-level hardware interface

## Repository Structure
```
HSA_Project/
├── docs/                    # Hardware datasheets and documentation
├── ros2_slam_ws/           # ROS2 SLAM documentation (code runs on Pi)
│   ├── Readme.md           # Detailed SLAM implementation by Liangyu Chen
│   └── src/
│       ├── basicsetup.md   # ROS2 setup instructions
│       └── sensor_publisher/  # Our custom sensor integration package
├── test_mcu_ws/            # Arduino firmware (PlatformIO)
│   └── arduino_integrated/  # Motor control + LiDAR interface
├── test_skin_ws/           # ROS1 workspace (runs in Docker)
│   └── src/
│       ├── skin_force_publisher/  # E-skin processing + motor control
│       └── lidar_publisher/       # LiDAR data relay to ROS2
└── set_permissions.sh      # USB/serial port permissions setup
```

## Module Details

### test_skin_ws (ROS1 Docker)
Main development workspace running in TUM ICS Docker container.

#### skin_force_publisher Package
- **force_publisher_node.cpp**: Reads 4 e-skin units, averages 3 force sensors per unit
- **FourCellForces.msg**: Custom message containing 4 force values
- **motor_control.py**: Subscribes to forces, sends motor commands via serial
- **force_publisher.launch**: Integrated launcher for complete system

Control mapping:
- Front touch → Backward movement
- Back touch → Forward movement  
- Left touch → Rotate right
- Right touch → Rotate left

#### lidar_publisher Package
- **lidar_publisher_node.py**: Receives "L:xxx" format from Arduino
- Publishes LaserScan messages for ROS2 SLAM compatibility
- 10cm offset correction applied to raw readings

### test_mcu_ws (Arduino)
PlatformIO project for Arduino UNO firmware.

- **Motor Control**: Receives "pwmA,dirA,pwmB,dirB" commands via serial
- **LiDAR Interface**: Uses official Garmin library (critical for proper operation)
- **Serial Protocol**: 9600 baud, bidirectional communication

### ros2_slam_ws (Documentation Only)
This folder contains documentation for the ROS2 SLAM system running natively on Raspberry Pi. The actual code resides on the Pi due to GPIO access requirements.

Key components documented:
- Extended Kalman Filter for IMU/encoder fusion
- Trajectory recording and visualization
- Single-point LiDAR adaptation for mapping
- Multi-sensor synchronization

## Running the System

### Prerequisites
1. Install Docker with TUM ICS ROS skin image
2. Connect e-skin hardware via USB
3. Upload Arduino firmware: `cd test_mcu_ws/arduino_integrated && pio run -t upload`
4. Run permissions script: `./set_permissions.sh`

### Launch Sequence

#### In Docker Container
```bash
# Terminal 1: E-skin and motor control
cd test_skin_ws
source devel/setup.bash
roslaunch skin_force_publisher force_publisher.launch

# Terminal 2: LiDAR relay
roslaunch lidar_publisher lidar_publisher.launch
```

#### On Raspberry Pi (separate)
```bash
# See ros2_slam_ws/basicsetup.md for full instructions
ros2 launch sensor_publisher slam_localization_launch.py
```

### Monitor System
```bash
# Force data from e-skin
rostopic echo /skin_forces

# LiDAR distance
rostopic echo /scan

# View all topics
rostopic list
```

## Implementation Status

### Completed
- E-skin force reading and averaging
- Force-proportional motor control
- Arduino serial communication
- LiDAR data acquisition
- IMU/encoder trajectory recording
- Basic LiDAR mapping

### Future Development Opportunities
- Obstacle avoidance using LiDAR data
- Autonomous return-to-origin using recorded trajectory
- Path planning and navigation
- Multi-robot coordination

## Development Guidelines
- Create feature branches for all development
- Test changes in isolated components before integration
- Document serial protocols and message formats
- Maintain separation between ROS1 and ROS2 components

---
HSA Project Team 2024