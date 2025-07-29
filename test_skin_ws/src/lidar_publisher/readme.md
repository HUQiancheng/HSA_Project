# lidar_publisher Package

## Overview
ROS node that interfaces with Arduino-based distance sensor and publishes data in LaserScan format for SLAM compatibility.

## Hardware Interface
- **Input**: Serial data from Arduino on `/dev/ttyACM0` (9600 baud)
- **Format**: `L:xxx` where xxx is distance in centimeters
- **Correction**: 10cm offset subtracted from raw readings

## Published Topics
- `/scan` (sensor_msgs/LaserScan): Single-point distance data wrapped in LaserScan format

## Parameters
- `~port` (default: `/dev/ttyACM0`): Serial port
- `~baudrate` (default: 9600): Serial baudrate
- `~frame_id` (default: `lidar_lite`): TF frame for sensor

## LaserScan Configuration
Since this is a single-point sensor, the LaserScan message is configured with:
- Zero angle range (min=max=0)
- Single range value in meters
- Intensity field contains raw cm reading

## Usage
```bash
roslaunch lidar_publisher lidar_publisher.launch
```

## Notes
- Valid range: 1-4000 cm
- Publishing rate: 20 Hz
- Compatible with modified SLAM algorithms that work with sparse distance data