## ROS Bridge and Launch Configuration

### Step 1: Setup ROS1 Environment (in Docker container)
```bash
# Source ROS1 Noetic
source /opt/ros/noetic/setup.bash

# Set ROS1 master URI
export ROS_MASTER_URI=http://localhost:11311
```

### Step 2: Configure ROS Bridge (on Raspberry Pi host)
```bash
# Enable X11 forwarding for Docker
xhost +local:docker

# Source the ROS1-ROS2 bridge
source ~/ros-humble-ros1-bridge/install/local_setup.bash

# Run the dynamic bridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Step 3: Network Configuration (for laptop communication)
```bash
# Set ROS2 domain ID (must match on all devices)
export ROS_DOMAIN_ID=0

# Use Fast-RTPS as DDS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Allow network discovery beyond localhost
export ROS_LOCALHOST_ONLY=0
```

### Step 4: Launch localization and SLAM System
```bash
# Navigate to ROS2 workspace (on Raspberry Pi)
cd ros2_ws

# Source the workspace
source install/setup.bash

# Launch localization and SLAM
ros2 launch sensor_publisher slam_localization_launch.py
```

**Note**: The `ros2_ws` mentioned here is the actual workspace on the Raspberry Pi (referred to as `ros2_slam_ws` in the repository documentation).