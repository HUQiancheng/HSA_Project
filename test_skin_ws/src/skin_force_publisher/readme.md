# E-Skin Force Publisher Example

## Overview
This example demonstrates how to interface with the TUM ICS e-skin sensors to read force data and publish it as a ROS topic. The system reads from 4 e-skin units, averages the 3 force sensors in each unit, and publishes a vector of 4 force values at 30 Hz.

## What This Example Does
The force publisher node performs these steps:
1. Loads the e-skin patch configuration
2. Establishes connection to 4 e-skin sensor units (IDs 1-4)
3. Continuously reads force data from each unit
4. Averages the 3 force sensors within each unit
5. Publishes the 4 averaged values as a FourCellForces message on `/skin_forces` topic

## System Architecture
```
[E-Skin Hardware] --USB--> [Skin Driver] --ROS--> [Force Publisher] --ROS--> [/skin_forces topic]
                                                          |
                                                          v
                                              Reads from config file
```

## Running the System

### Prerequisites
Run the permissions setup script once:
```bash
sudo ./set_permissions.sh
```

### Step 1: Start ROS Master
Open a terminal and run:
```bash
roscore
```

### Step 2: Launch Complete E-Skin System
In a new terminal:
```bash
cd /workspaces/HSA_Project/test_skin_ws
source devel/setup.bash
roslaunch skin_force_publisher force_publisher.launch
```

This integrated launch file will:
1. **Connect to e-skin hardware** (equivalent to the old skin driver step)
2. **Start force publisher node** (reads and publishes sensor data)
3. **Start motor controller** (translates forces to motor commands)

You should see output like:
```
Num of skin cells: 4
Connected
Successfully loaded patch with 4 cells
Cell IDs in patch:
  Index 0 -> Cell ID 1
  Index 1 -> Cell ID 2
  Index 2 -> Cell ID 3
  Index 3 -> Cell ID 4
Forces: [0.0039, 0.0049, 0.0029, 0.0039]
✅ Motor controller ready, listening to /skin_forces
```

### Step 3: Monitor the System (Optional)
In another terminal, view the real-time force data:
```bash
rostopic echo /skin_forces
```

Output format:
```yaml
data: [0.00390625, 0.00488281, 0.00292969, 0.00390625]
---
```

Each value represents the averaged force from one e-skin unit. The array indices correspond to:
- Index 0: Cell ID 1 (Front)
- Index 1: Cell ID 2 (Left)
- Index 2: Cell ID 3 (Back)  
- Index 3: Cell ID 4 (Right)

## Testing the Sensors
To verify the system is working correctly:
1. Gently press on one of the e-skin units
2. Watch the corresponding value in the array increase
3. Release pressure and see the value return to baseline

The force values are in uncalibrated units. Typical baseline values are around 0.003-0.005, and can increase to 0.1 or higher with moderate pressure.

## Understanding the Code
The main source files are:

- **`src/force_publisher_node.cpp`**: Main C++ node for data processing
- **`scripts/motor_control.py`**: Motor control functionality
- **`scripts/motor_test.py`**: Motor testing utilities
- **`msg/FourCellForces.msg`**: Custom message definition

Key components:
- **Patch Loading**: Uses TfMarkerDataPatch class to load sensor configuration
- **Data Connection**: Establishes real-time data stream from skin driver
- **Force Averaging**: Each e-skin unit has 3 force sensors that are averaged
- **Publishing Loop**: Runs at 30 Hz, continuously reading and publishing data

## Troubleshooting

### No data published
- Check that ROS core is running (`roscore`)
- Verify the e-skin hardware is connected and powered
- Ensure the launch file starts successfully without errors

### All zeros in output
- The skin driver may not be detecting the sensors
- Check USB connection and power to the e-skin units
- Verify sensor configuration is loaded correctly

### Build errors
- Ensure all dependencies are installed
- Clean and rebuild: `cd /workspaces/HSA_Project/test_skin_ws && rm -rf build devel && catkin_make`
- Source the workspace: `source devel/setup.bash`

### Permission issues
- Ensure Python scripts have execute permissions: `chmod +x scripts/*.py`
- Check USB device permissions for hardware connections

## Adapting This Example
To use this code as a starting point for your own applications:
1. Copy the package structure
2. Modify the data processing logic in the main loop
3. Change the output message type if needed (see `msg/FourCellForces.msg`)
4. Adjust the publishing rate as required

Remember that the e-skin sensors can also provide proximity, acceleration, and temperature data if needed for your application.