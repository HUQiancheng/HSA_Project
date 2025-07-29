# Robotic Localization and SLAM System Documentation

## System Overview

This documentation covers a sophisticated robotic localization and simultaneous localization and mapping (SLAM) system developed by Liangyu Chen for deployment on Raspberry Pi 5. The system demonstrates advanced sensor fusion techniques combining wheel encoders, inertial measurement units (IMU), and distance sensors to achieve autonomous navigation and environment mapping capabilities.

**Important Note**: This workspace represents a snapshot of code that was developed and tested directly on Raspberry Pi 5 hardware. The code in this directory is provided for documentation and educational purposes only and cannot be executed without the complete ROS2 workspace structure and hardware setup that exists on the original Raspberry Pi system.

## Technical Architecture

Liangyu Chen's system implements a layered architecture following modern robotics best practices:

**Sensor Layer**: Hardware interfaces that convert physical measurements into digital data streams
**Processing Layer**: Individual nodes that handle sensor-specific data processing and filtering  
**Fusion Layer**: Multi-sensor data integration using Extended Kalman Filtering techniques
**Mapping Layer**: Simultaneous Localization and Mapping (SLAM) algorithm implementation
**Visualization Layer**: Real-time system monitoring and trajectory recording

## Detailed Code Analysis

### 1. Encoder Node (`encoder_node1.py`)

**Purpose**: This node serves as the foundation of the robot's proprioceptive sensing system, converting raw wheel rotation data into precise odometry information.

**Technical Implementation**: The encoder node implements differential drive kinematics to compute robot motion from wheel encoder pulses. It uses GPIO interrupt handling to capture encoder signals in real-time, then applies mathematical transformations to convert pulse counts into linear and angular velocities.

**Key Features**:
- Real-time GPIO pulse counting with debouncing
- Differential drive kinematics calculations  
- Dynamic covariance adjustment based on motion state
- TF frame broadcasting for coordinate system integration
- Configurable wheel parameters and resolution settings

**Data Flow**: Raw encoder pulses → Pulse counting → Distance calculation → Velocity computation → Odometry message publishing to `/wheel_odom` topic

### 2. IMU Node (`imu_node.py`)

**Purpose**: This node interfaces with the BNO085 inertial measurement unit to provide orientation, angular velocity, and linear acceleration data essential for robust robot localization.

**Technical Implementation**: The node communicates with the BNO085 sensor via I2C protocol, retrieving fusion-processed orientation data alongside raw accelerometer and gyroscope measurements. It publishes both raw sensor data for external filtering and processed orientation information.

**Key Features**:
- I2C communication with BNO085 sensor
- Multi-rate data acquisition (accelerometer, gyroscope, magnetometer)
- Quaternion orientation output for gimbal-lock-free representation  
- Separate raw data stream for external complementary filtering
- Configurable publication rates and sensor parameters

**Data Flow**: BNO085 sensor → I2C data retrieval → Message formatting → Publication to `/imu/data_raw` and magnetometer topics

### 3. LiDAR Node (`lidar_node.py`)

**Purpose**: This node processes single-point distance sensor data and formats it into LaserScan messages compatible with standard SLAM algorithms, despite the hardware limitation of having only one distance measurement point.

**Technical Implementation**: The node reads distance measurements from a serial-connected sensor and publishes them in the standard LaserScan message format. While traditional SLAM requires 360-degree scanning data, this implementation provides the foundation for modified SLAM approaches that can work with sparse distance data.

**Key Features**:
- Serial communication with distance sensor
- LaserScan message formatting for SLAM compatibility
- Configurable measurement ranges and thresholds
- Real-time distance data processing and validation
- Error handling for invalid measurements

**Data Flow**: Distance sensor → Serial data → Parsing and validation → LaserScan message → Publication to `/scan` topic

### 4. Path Publisher (`path_publisher.py`)

**Purpose**: This node creates a persistent record of the robot's movement trajectory by accumulating position data from the fused odometry stream, enabling visualization of the complete robot path in RViz.

**Technical Implementation**: The node subscribes to the filtered odometry output from the Extended Kalman Filter and continuously builds a path message by storing each position update. This creates a breadcrumb trail showing everywhere the robot has traveled.

**Key Features**:
- Continuous trajectory accumulation from filtered odometry
- Path message construction with proper frame references
- Real-time trajectory visualization support
- Memory-efficient path storage and management

**Data Flow**: Filtered odometry (`/odometry/odom_filtered`) → Position extraction → Path accumulation → Trajectory publishing to `/trajectory` topic

### 5. EKF Configuration (`ekf.yaml`)

**Purpose**: This configuration file defines the parameters for the Extended Kalman Filter that performs multi-sensor data fusion, combining wheel encoder and IMU data into a unified, accurate state estimate.

**Technical Implementation**: The configuration specifies which dimensions of each sensor's data should be incorporated into the state estimate, along with process noise and measurement noise parameters that define the filter's trust levels for different data sources.

**Key Configuration Elements**:
- Sensor input topic mappings and data dimension selection
- Process noise covariance matrices defining system uncertainty
- Initial state covariance matrices for filter initialization  
- Frame reference definitions for coordinate system consistency
- Temporal parameters for data synchronization and timeout handling

**Critical Parameters**:
- `odom0_config`: Defines which wheel odometry dimensions to use (position, velocity, orientation)
- `imu0_config`: Specifies which IMU measurements to incorporate (angular velocity, linear acceleration)
- Covariance matrices: Quantify measurement uncertainty and process noise levels

### 6. System Launch File (`slam_localization_launch.py`)

**Purpose**: This orchestration file coordinates the startup and configuration of all system components in the correct sequence, ensuring proper inter-node communication and parameter passing.

**Technical Implementation**: The launch file uses ROS2's launch system to start multiple nodes simultaneously while managing their dependencies, parameter loading, and topic remapping. It represents the "conductor" that brings together all the individual components into a functioning robotic system.

**System Startup Sequence**:
1. Hardware sensor nodes initialization (IMU, encoders, distance sensor)
2. Signal processing nodes startup (IMU filtering, coordinate transforms)
3. Data fusion layer activation (Extended Kalman Filter)
4. SLAM algorithm initialization with optimized parameters
5. Visualization and monitoring node deployment

## System Integration and Data Flow

The complete system demonstrates sophisticated engineering in how these components work together. Sensor data flows from hardware through processing nodes into the fusion algorithm, which produces a unified state estimate that feeds into the SLAM system for environment mapping.

The key innovation in Liangyu Chen's implementation lies in adapting traditional SLAM algorithms to work with single-point distance sensors rather than full 360-degree laser scanners. This required careful parameter tuning and algorithmic modifications to extract meaningful spatial information from limited sensor data.

## Hardware and Software Integration Challenges

Liangyu Chen's implementation overcame several significant technical challenges:

**Multi-ROS Version Compatibility**: Successfully integrated ROS1 and ROS2 components on the same system
**SLAM Toolbox Installation**: Resolved complex dependency issues for SLAM algorithms on ARM architecture
**Sensor Fusion Optimization**: Tuned EKF parameters for optimal performance with specific hardware configuration
**Single-Point SLAM**: Developed techniques to perform mapping with minimal sensor data

This system represents a sophisticated example of modern robotic engineering, demonstrating how careful software architecture and parameter tuning can achieve complex autonomous behaviors even with limited hardware resources.
