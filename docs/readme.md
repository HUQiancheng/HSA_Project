# E-Skin Force Publisher Example

## Overview
This example demonstrates how to interface with the TUM ICS e-skin sensors to read force data and publish it as a ROS topic. The system reads from 4 e-skin units, averages the 3 force sensors in each unit, and publishes a vector of 4 force values at 30 Hz.

## What This Example Does
The force publisher node performs these steps:
1. Loads the e-skin patch configuration from `launch/configs/default.xml`
2. Establishes connection to 4 e-skin sensor units (IDs 1-4)
3. Continuously reads force data from each unit
4. Averages the 3 force sensors within each unit
5. Publishes the 4 averaged values as a Float64MultiArray on `/skin_forces` topic

## System Architecture
```
[E-Skin Hardware] --USB--> [Skin Driver] --ROS--> [Force Publisher] --ROS--> [/skin_forces topic]
                                                          |
                                                          v
                                              Reads from config file
                                              (launch/configs/default.xml)
```

## Running the Example

### Step 1: Start ROS Master
Open a terminal and run:
```bash
roscore
```

### Step 2: Connect to E-Skin Hardware
In a new terminal, launch the skin driver:
```bash
roslaunch tum_ics_skin_driver_events skin_driver_ftdi.launch FTDI_SERIAL:=FT6B8EHV
```

You should see output confirming 4 skin cells are detected:
```
Num of skin cells: 4
Connected
```

### Step 3: Launch the Force Publisher
In another terminal:
```bash
cd /workspaces/HSA_Project/test_skin_ws
source devel/setup.bash
roslaunch launch/skin_force_publisher/force_publisher.launch
```

You should see output like:
```
Successfully loaded patch with 4 cells
Cell IDs in patch:
  Index 0 -> Cell ID 1
  Index 1 -> Cell ID 2
  Index 2 -> Cell ID 3
  Index 3 -> Cell ID 4
Forces: [0.0039, 0.0049, 0.0029, 0.0039]
```

### Step 4: Monitor the Published Data
In a fourth terminal, view the real-time force data:
```bash
rostopic echo /skin_forces
```

Output format:
```yaml
data: [0.00390625, 0.00488281, 0.00292969, 0.00390625]
---
```

Each value represents the averaged force from one e-skin unit. The array indices correspond to:
- Index 0: Cell ID 1
- Index 1: Cell ID 2  
- Index 2: Cell ID 3
- Index 3: Cell ID 4

## Testing the Sensors
To verify the system is working correctly:
1. Gently press on one of the e-skin units
2. Watch the corresponding value in the array increase
3. Release pressure and see the value return to baseline

The force values are in uncalibrated units. Typical baseline values are around 0.003-0.005, and can increase to 0.1 or higher with moderate pressure.

## Understanding the Code
The main source file is at `src/skin_force_publisher/src/force_publisher_node.cpp`. Key components:

- **Patch Loading**: Uses TfMarkerDataPatch class to load sensor configuration
- **Data Connection**: Establishes real-time data stream from skin driver
- **Force Averaging**: Each e-skin unit has 3 force sensors that are averaged
- **Publishing Loop**: Runs at 30 Hz, continuously reading and publishing data

## Troubleshooting

### No data published
- Check that all 3 systems are running (roscore, skin driver, force publisher)
- Verify the e-skin hardware is connected and powered
- Check that `default.xml` exists at the expected path

### All zeros in output
- The skin driver may not be detecting the sensors
- Try running `cf on` in the skin driver terminal to enable color feedback
- Check USB connection and power to the e-skin units

### Build errors
- Ensure all dependencies are installed
- Clean and rebuild: `cd /workspaces/HSA_Project/test_skin_ws && rm -rf build devel && catkin_make`

## Adapting This Example
To use this code as a starting point for your own applications:
1. Copy the package structure
2. Modify the data processing logic in the main loop
3. Change the output message type if needed
4. Adjust the publishing rate as required

Remember that the e-skin sensors can also provide proximity, acceleration, and temperature data if needed for your application.
