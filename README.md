# HSA Project - E-Skin Controlled Vehicle with IMU Trajectory Recording

## Project Overview
This project implements a vehicle control system using TUM ICS electronic skin (e-skin) sensors for directional control, combined with IMU-based trajectory recording. The e-skin sensors detect force inputs that are translated into movement commands for the vehicle.

## Repository Structure
```
HSA_Project/
├── docs/               # Collected documentation and research materials
├── skin_tutorial/      # TUM ICS skin tutorial examples (reference only)
└── test_skin_ws/       # Main development workspace - START HERE
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

### Sensor System
- **4x HEX-o-skin Units** (e-skin by TUM ICS)
  - Layout: Front, Back, Left, Right
  - ID Assignment: Front=1, Left=2, Back=3, Right=4
  - Function: Force sensing (other sensing capabilities not utilized)
- **IMU**: Adafruit 9-DOF Orientation IMU (BNO085)

### Actuators
- **Motors**: 4x DC Gear Motors (2 left, 2 right)
- **Motor Drivers**: 2x L298N (one for each side)

### Additional Hardware
- 1x Arduino development board (spare)

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

## Current Progress

### Completed
- E-skin force sensor data acquisition
- ROS topic publishing functionality
- Custom message type: `FourCellForces.msg`
  ```
  # Averaged force values from 4 skin cells
  # Index 0 = Cell ID 1
  # Index 1 = Cell ID 2
  # Index 2 = Cell ID 3
  # Index 3 = Cell ID 4
  float64[4] forces
  ```
- Published on topic: `/skin_forces`

### Monitoring Force Data
```bash
# In a terminal, view real-time force data:
rostopic echo /skin_forces

# Output format:
# data: [0.00390625, 0.00488281, 0.00292969, 0.00390625]
# ---
```

### To Be Implemented
- Motor control system development and integration
- IMU trajectory recording functionality
- System integration

## Project Components
- **E-Skin Interface** (Lukas): Force sensor data acquisition and processing ✓
- **Vehicle Control** (TBD): Translates sensor inputs to movement commands  
- **IMU Integration** (TBD): Records and processes trajectory data
- **System Integration** (TBD): Combines all components into working system

## Next Steps
For development instructions, see `test_skin_ws/README.md`