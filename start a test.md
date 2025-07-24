T1(Terminal 1):

ross # dev defined command
roscore

T2:
ross 
roslaunch tum_ics_skin_driver_events skin_driver_ftdi.launch FTDI_SERIAL:=FT6B8EHV

T3:
ross
cd test_skin_ws/
source devel/setup.bash
roslaunch skin_force_publisher force_publisher.launch 

---

T4:
ross
cd test_skin_ws/
source devel/setup.bash 
sudo chmod 666 /dev/ttyACM0
ls -l /dev/ttyACM0
rosrun skin_force_publisher motor_control.py 


T5:
ross
cd test_skin_ws/
source devel/setup.bash 
