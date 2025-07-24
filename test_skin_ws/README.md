# Test Skin Workspace - Development Guide

## Workspace Overview
This is the main ROS workspace for the HSA e-skin controlled vehicle project. The workspace follows standard ROS conventions with launch files organized within their respective packages.

## Directory Structure
```
test_skin_ws/
├── build/                           # Auto-generated build files (git-ignored)
├── devel/                           # Auto-generated development files (git-ignored)
└── src/                            # Source packages
    ├── CMakeLists.txt              # Catkin top-level CMake (auto-generated)
    ├── lidar_publisher/            # LiDAR data publisher package
    │   ├── launch/                 # Package-specific launch files
    │   │   └── lidar_publisher.launch
    │   └── scripts/                # Python scripts
    │       └── lidar_publisher_node.py
    └── skin_force_publisher/       # E-skin force data publisher package
        ├── launch/                 # Package-specific launch files
        │   └── force_publisher.launch
        ├── msg/                    # Custom message definitions
        │   └── FourCellForces.msg
        ├── scripts/                # Python scripts
        │   ├── motor_control.py
        │   ├── motor_test.py
        │   └── readme.md
        └── src/                    # C++ source files
            └── force_publisher_node.cpp
```

## Development Setup

### Prerequisites
Before starting development, ensure you have:
1. Docker with the TUM ICS ROS skin image installed
2. VS Code with Dev Containers extension
3. The e-skin hardware connected via USB

### Initial Setup
Open VS Code in the workspace directory and rebuild the container:
```bash
cd test_skin_ws
code .
# Press Ctrl+Shift+P and run "Dev Containers: Rebuild Container"
```

### Building the Workspace
Inside the VS Code terminal (which runs in the Docker container):
```bash
# Source ROS environment
ross

# Build all packages
catkin_make

# Source the workspace
source devel/setup.bash
```

## Package Overview

### `skin_force_publisher`
- **Purpose**: E-skin force sensor data processing and publishing
- **Key Files**:
  - `FourCellForces.msg`: Custom message for 4-cell force data
  - `force_publisher_node.cpp`: C++ node for data processing
  - `motor_control.py`, `motor_test.py`: Motor control scripts
- **Launch**: `roslaunch skin_force_publisher force_publisher.launch`

### `lidar_publisher`
- **Purpose**: LiDAR sensor data processing
- **Key Files**:
  - `lidar_publisher_node.py`: Python node for LiDAR data
- **Launch**: `roslaunch lidar_publisher lidar_publisher.launch`

## Creating New Packages
When adding new functionality, create packages in the `src/` directory:
```bash
cd src
catkin_create_pkg your_package_name roscpp std_msgs [other_dependencies]
```

Follow these conventions:
- Package names use lowercase with underscores
- Place C++ source files in `src/`
- Place Python scripts in `scripts/`
- Place message definitions in `msg/`
- Place service definitions in `srv/`
- Keep launch files in the package's `launch/` directory

## Working with E-Skin Data

The `skin_force_publisher` package demonstrates how to:
- Process e-skin sensor force data
- Publish data using custom `FourCellForces.msg` message type
- Interface with motor control systems

### Custom Message Format
```
# FourCellForces.msg
# Averaged force values from 4 skin cells
# Index 0 = Cell ID 1 (Front)
# Index 1 = Cell ID 2 (Left)  
# Index 2 = Cell ID 3 (Back)
# Index 3 = Cell ID 4 (Right)
float64[4] forces
```

## Running the System

### Basic Testing Workflow
1. **Start ROS core**: `roscore`
2. **Launch e-skin publisher**: `roslaunch skin_force_publisher force_publisher.launch`
3. **Launch LiDAR (if needed)**: `roslaunch lidar_publisher lidar_publisher.launch`
4. **Monitor data**: `rostopic echo /skin_forces`

### Available Topics
- `/skin_forces`: E-skin force sensor data (FourCellForces message)

## Development Tips

### Building and Testing
- Always clean and rebuild after making CMakeLists.txt changes: `rm -rf build devel && catkin_make`
- Use `ROS_INFO`, `ROS_WARN`, and `ROS_ERROR` for debugging instead of printf/cout
- Test individual nodes before system integration

### Common Commands
```bash
# List all topics
rostopic list

# Monitor specific topic
rostopic echo /skin_forces

# Check node graph
rqt_graph

# View message structure
rosmsg show skin_force_publisher/FourCellForces
```

## Troubleshooting
- Ensure all Python scripts have execute permissions: `chmod +x scripts/*.py`
- Check USB device permissions for hardware connections
- Source the workspace setup after each build: `source devel/setup.bash`

## Need Help?
- For e-skin specific questions, check the `skin_force_publisher` package
- For ROS basics, see the official ROS tutorials
- For project-specific questions, contact the relevant component owner