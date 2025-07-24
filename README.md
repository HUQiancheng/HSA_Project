# HSA Project - E-Skin Controlled Vehicle with IMU Trajectory Recording

## Project Overview
This project implements a vehicle control system using TUM ICS electronic skin (e-skin) sensors for directional control, combined with IMU-based trajectory recording and LiDAR sensing. The e-skin sensors detect force inputs that are translated into movement commands for the vehicle.

## Repository Structure
```
HSA_Project/
├── docs/               # Collected documentation and research materials
├── skin_tutorial/      # TUM ICS skin tutorial examples (reference only)
├── test_mcu_ws/        # Arduino workspace
│   └── arduino_integrated/  # Arduino code (PlatformIO project)
├── test_skin_ws/       # Main ROS development workspace - START HERE
├── set_permissions.sh  # One-time system permissions setup
└── build/              # Build artifacts
```

## Important Development Guidelines
### ⚠️ BEFORE YOU START
1. **Never push directly to main/master branch**
2. **Always create your own feature branch for development**
3. **Submit pull requests for code review before merging**

### Getting Started
```bash
# Clone the repository
git clone [repository-url]

# Set up permissions (one-time setup)
sudo ./set_permissions.sh

# Create your feature branch
git checkout -b feature/your-feature-name

# Navigate to the development workspace
cd test_skin_ws

# Open VS Code from here (not from the top level)
code .
```

## Hardware Configuration

### Main Platform
- **Controller**: Raspberry Pi 5 (aarch64)
- **Operating System**: Debian Bookworm (Raspberry Pi OS)

### Arduino Microcontroller System
- **Arduino Board**: Handles low-level sensor and motor control
- **Functions**: Motor control, LiDAR data acquisition, serial communication
- **Connection**: USB serial (/dev/ttyACM0) to Raspberry Pi
- **Code Location**: `test_mcu_ws/arduino_integrated/` (PlatformIO project)

### Sensor System
- **4x HEX-o-skin Units** (e-skin by TUM ICS)
  - Layout: Front, Back, Left, Right
  - ID Assignment: Front=1, Left=2, Back=3, Right=4
  - Function: Force sensing (other sensing capabilities not utilized)
- **LiDAR Sensor**: Distance measurement and obstacle detection
- **IMU**: Adafruit 9-DOF Orientation IMU (BNO085)

### Actuators
- **Motors**: 4x DC Gear Motors (2 left, 2 right)
- **Motor Drivers**: 2x L298N (one for each side)

## Software Environment
### Docker Container (provided by instructor)
- **Base OS**: Ubuntu 20.04
- **ROS Version**: Noetic
- **Purpose**: Integration of e-skin with ROS system

## Control Logic
The vehicle responds to touches on different e-skin units with specific movements:

| Touched Unit | ID | Movement Behavior |
|--------------|-----|-------------------|
| Front | 1 | Motors reverse, move backward |
| Left | 2 | Turn right in place (left motors forward, right motors reverse) |
| Back | 3 | Motors forward, move forward |
| Right | 4 | Turn left in place (right motors forward, left motors reverse) |

*Note: Motor speed is adjusted based on force sensor feedback*

## Quick Start Guide

### Prerequisites
Run the permissions setup script once:
```bash
sudo ./set_permissions.sh
```

### System Startup (Every Test Session)
```bash
# Terminal 1: Start ROS Master
roscore

# Terminal 2: Launch E-Skin System (integrated)
cd test_skin_ws
source devel/setup.bash
roslaunch skin_force_publisher force_publisher.launch

# Terminal 3: Launch LiDAR System
roslaunch lidar_publisher lidar_publisher.launch
```

### Monitoring System Data
```bash
# View real-time force data
rostopic echo /skin_forces

# View LiDAR distance readings  
rostopic echo /lidar_distance

# List all active topics
rostopic list
```

## Current Progress

### ✅ Completed
- **E-skin Interface**: Force sensor data acquisition and processing
- **Arduino Integration**: Low-level motor control and LiDAR data collection
- **ROS Integration**: Topic publishing and communication system
- **Motor Control System**: Force-to-movement translation with PWM control
- **LiDAR System**: Distance sensing capabilities
- **Custom Message Types**:
  - `FourCellForces.msg` for e-skin data
  ```
  # Averaged force values from 4 skin cells
  # Index 0 = Cell ID 1 (Front)
  # Index 1 = Cell ID 2 (Left)
  # Index 2 = Cell ID 3 (Back)
  # Index 3 = Cell ID 4 (Right)
  float64[4] forces
  ```

### 📊 Active ROS Topics
- `/skin_forces` - E-skin force data (FourCellForces message)
- `/lidar_distance` - LiDAR sensor readings

### 🔄 In Development
- **IMU Integration**: Trajectory recording functionality
- **System Optimization**: Performance tuning and reliability improvements

## Available ROS Packages

### `skin_force_publisher`
- **Function**: Complete e-skin system integration
- **Components**: Force data processing, motor control, hardware communication
- **Launch**: `roslaunch skin_force_publisher force_publisher.launch`
- **Features**: Includes skin driver connection and motor control integration

### `lidar_publisher` 
- **Function**: LiDAR data processing and publishing
- **Launch**: `roslaunch lidar_publisher lidar_publisher.launch`
- **Output**: Distance measurements on `/lidar_distance` topic

## Project Components Status
- **E-Skin Interface** (Lukas): Force sensor data acquisition and processing ✅
- **Arduino Integration**: Low-level control and communication ✅
- **Motor Control System**: Force-to-movement translation ✅
- **LiDAR System**: Environmental sensing ✅
- **IMU Integration**: Trajectory recording 🔄
- **System Integration**: Component coordination and optimization 🔄

## Troubleshooting

### Common Issues
- **Permission denied**: Re-run `sudo ./set_permissions.sh`
- **No data published**: Check hardware connections and launch sequence
- **Build errors**: Clean rebuild with `rm -rf build devel && catkin_make`

### Hardware Verification
```bash
# Check USB connections
ls -l /dev/ttyACM*

# Verify ROS topics
rostopic list

# Monitor system status
rqt_graph
```

## Next Steps
For detailed development instructions and package-specific documentation, see:
- `test_skin_ws/README.md` - ROS workspace development guide
- `test_skin_ws/src/skin_force_publisher/readme.md` - E-skin system details
- Individual package documentation in respective directories

## Architecture Notes
- **Unified Arduino Control**: Single Arduino handles both motor control and LiDAR sensing
- **Integrated Launch Files**: Simplified startup process with automatic component coordination  
- **Modular Design**: Each ROS package handles specific functionality while maintaining clean interfaces