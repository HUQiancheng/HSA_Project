## open env
source myenv/bin/activate
source ~/ros2_humble/myenv/bin/activate
source ~/env/bin/activate
python3 -m venv --system-site-packages myenv



## run ros bridge
# in ros1 container
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# local
xhost +local:docker

source ~/ros2_humble/install/setup.bash 
source ~/ros-humble-ros1-bridge/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

echo $ROS_MASTER_URI
echo $ROS_IP
echo $ROS_HOSTNAME

£ communicate with laptop ros
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

test:
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener



£ test 
  rocker --x11 --user --privileged --persist-image \
         --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
         'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'


  source /opt/ros/noetic/setup.bash
  roscore

ROS1 Noetic 
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker

ROS2 Humble 
  ros2 run demo_nodes_cpp listener

## colcon build
colcon build --symlink-install
colcon build --packages-select sensor_publisher --symlink-install
colcon build --packages-select sensor_publisher robot_localization --symlink-install --merge-install


## I2C
sudo i2cdetect -y 1

source install/setup.bash
ros2 launch sensor_publisher slam_localization_launch.py 