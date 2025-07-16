# How to run the tutorial

## Setup

1) Install docker, see `install-docker.md`.
2) Install vscode, see `install-vscode.md`.
3) Deploy the TUM ICS ROS skin image to docker, see `deploy-docker-image.md`
4) Install the udev rules for the USB skin interface, see `skin-udev-rules.md`



## Run the tutorial 


### Open the workspace in the dev container

Press `CTRL+SHIFT+P` and run `Dev Containers: Rebuild Container`.

Now all terminals in VS code will run inside the docker container and you have
access to ROS and the skin API.


### Build the workspace

Open a terminal in vscode and run:

```bash
# Source the ROS environment
ross

# Build command
catkin_make
```


### Start the roscore

Open a terminal in vscode and run:

```bash
# Source the ROS environment
ross

# Run the roscore (only once per PC)
roscore
```


### Start the skin driver

Open another terminal in vscode and run:

```bash
# Source the ROS environment
ross

# Run the skin driver // sudo dmesg 
roslaunch tum_ics_skin_driver_events skin_driver_ftdi.launch FTDI_SERIAL:=FT6B8EHV
```

You can now execute commands in the skin driver, e.g. 
- `cf on` to turn on the color feedback
- `cf off` to turn off the color feedback
- `h` to display all available commands
- `red` to turn all LEDs red
- `q` to cleanly exit the application



### Start the patch configurator

Open another terminal in vscode and run:

```bash
# Source the ROS workspace
. devel/setup.bash

# Run the patch setup GUI
roslaunch skin_tutorial full_config.launch
```

The command will open a GUI to configure your patch. Press two times the button 
`Capture` and the programm will construct the patches for you.

You configure and can save your patch in `Step 2: Custom Config`. Press `Save`
and select a path to save your patch config `.xml` file.

The standard path would be `/workspaces/skin_tut_ws/src/skin_tutorial/launch/configs/default.xml`.


You can use the same GUI also to load a config file and display it. Just press
the `Load` button in `Step 1: Auto config`.


### Launch the tutoral node and display the patch

Open another terminal in vscode and run:

```bash
# Source the ROS workspace
. devel/setup.bash

# Run the tutorial example
roslaunch skin_tutorial load_and_view_patch.launch
```

We can use RVIZ to visualize the patch in 3D space:

```bash
# Source the ROS workspace
. devel/setup.bash

# Run RVIZ to visualize the patch
roslaunch skin_tutorial rviz.launch 
```