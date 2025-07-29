# test_mcu_ws - Arduino Control Module

## Overview
Arduino UNO firmware providing low-level hardware control for motors and distance sensor. Communicates with Raspberry Pi via serial.

## Hardware Interface
- **Microcontroller**: Arduino UNO
- **Motor Driver**: L298N (controls 2 motors)
  - Motor A: ENA=5, IN1=6, IN2=7
  - Motor B: ENB=9, IN3=10, IN4=11
- **Distance Sensor**: Garmin LIDAR-Lite v4 (I2C)

## Serial Protocol (9600 baud)
**Input (Motor Control)**:
```
pwmA,dirA,pwmB,dirB
```
- pwmA/B: 0-255 (speed)
- dirA/B: 0/1 (direction)
- Example: `200,1,200,0` (both motors forward at ~78% speed)

**Output (Distance Data)**:
```
L:xxx
```
- Sends distance in cm every 100ms
- Example: `L:150` (150cm distance)

## Structure
```
test_mcu_ws/
└── arduino_integrated/
    ├── include/         # Header files
    ├── src/            # Implementation
    └── platformio.ini  # PlatformIO config
```

## Build & Upload
```bash
cd test_mcu_ws/arduino_integrated
pio run -t upload
```

## Critical Configuration
The official Garmin LIDAR-Lite library is **essential** for proper sensor operation:
```ini
lib_deps = garmin/LIDAR-Lite@^3.0.5
```

## Integration with ROS
- Motor commands come from `test_skin_ws/skin_force_publisher/scripts/motor_control.py`
- Distance data consumed by `test_skin_ws/lidar_publisher/scripts/lidar_publisher_node.py`