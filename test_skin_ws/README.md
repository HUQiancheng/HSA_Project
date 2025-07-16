# Test Skin Workspace - Development Guide

## Workspace Overview
This is the main ROS workspace for the HSA e-skin controlled vehicle project. The workspace follows standard ROS conventions with clear separation between source packages, launch files, and configurations.

## Directory Structure
```
test_skin_ws/
├── build/                           # Auto-generated build files (git-ignored)
├── devel/                           # Auto-generated development files (git-ignored)
├── launch/                          # System-level launch files and configurations
│   ├── configs/                     # Configuration files
│   │   └── default.xml             # E-skin patch configuration (DO NOT MODIFY)
│   └── skin_force_publisher/        # Example e-skin force publisher
└── src/                            # Source packages
    ├── CMakeLists.txt              # Catkin top-level CMake (auto-generated)
    └── skin_force_publisher/        # E-skin force data publisher package
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

## Creating New Packages
When adding new functionality, create packages in the `src/` directory:
```bash
cd src
catkin_create_pkg your_package_name roscpp std_msgs [other_dependencies]
```

Follow these conventions:
- Package names use lowercase with underscores
- Place C++ source files in `src/`
- Place message definitions in `msg/`
- Place service definitions in `srv/`
- Keep launch files at the workspace level under `launch/your_package_name/`

## Working with E-Skin Data
For an example of how to interface with the e-skin sensors and process force data, see the provided example at `launch/skin_force_publisher/README.md`. This demonstrates:
- Loading e-skin patch configurations
- Reading force sensor data
- Publishing processed data as ROS topics

## Testing Your Code
Always test in this order:
1. Start roscore: `roscore`
2. Connect to e-skin hardware: `roslaunch tum_ics_skin_driver_events skin_driver_ftdi.launch FTDI_SERIAL:=FT6B8EHV`
3. Launch your nodes
4. Monitor topics with `rostopic echo` or `rqt`

## Important Notes
- The `launch/configs/default.xml` file contains the e-skin sensor configuration. This file is referenced by absolute path in the code and must not be moved or modified.
- Always clean and rebuild after making CMakeLists.txt changes: `rm -rf build devel && catkin_make`
- Use `ROS_INFO`, `ROS_WARN`, and `ROS_ERROR` for debugging instead of printf/cout

## Need Help?
- For e-skin specific questions, check the skin_force_publisher example
- For ROS basics, see the official ROS tutorials
- For project-specific questions, contact the relevant component owner